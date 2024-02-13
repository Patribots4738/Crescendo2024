package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive;
import frc.robot.commands.PieceControl;
import frc.robot.commands.ShooterCalc;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.CalibrationControl;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PatriBoxController;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import monologue.Annotations.Log;
import frc.robot.util.PIDNotConstants;
import frc.robot.util.PIDTunerCommands;
import monologue.Logged;

public class RobotContainer implements Logged {
    
    private final PatriBoxController driver;
    private final PatriBoxController operator;

    private Swerve swerve;
    private final Intake intake;

    private Limelight limelight;
    private final LedStrip ledStrip;
    private final Climb climb;
    private Indexer triggerWheel;
    private Pivot pivot;
    private Shooter shooter;
    private Claw claw;
    private Elevator elevator;
    private ShooterCalc shooterCalc;
    private PieceControl pieceControl;
    private CalibrationControl calibrationControl;
    private PIDTunerCommands PIDTuner;
    
    @Log
    public static Pose3d[] components3d = new Pose3d[5];

    @Log
    public static Pose3d[] desiredComponents3d = new Pose3d[5];

    @Log
    public static Pose3d[] notePose3ds = new Pose3d[12];
    
    public RobotContainer() {
        
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);
        
        intake = new Intake();
        climb = new Climb();
        swerve = new Swerve();
        limelight = new Limelight(swerve::getPose);
        ledStrip = new LedStrip(swerve::getPose);
        triggerWheel = new Indexer();

        shooter = new Shooter();
        elevator = new Elevator();
        claw = new Claw();
        
        pivot = new Pivot();
        incinerateMotors();
        
        shooterCalc = new ShooterCalc(shooter, pivot);
        
        PIDTuner = new PIDTunerCommands(new PIDNotConstants[] {
            pivot.getPIDNotConstants(),
            shooter.getPIDNotConstants(),
            elevator.getPIDNotConstants(),
            climb.getPidNotConstants()
        });

        pieceControl = new PieceControl(
                intake,
                triggerWheel,
                elevator,
                claw,
                shooterCalc);

        calibrationControl = new CalibrationControl(shooterCalc);

        limelight.setDefaultCommand(Commands.run(() -> {
            // Create an "Optional" object that contains the estimated pose of the robot
            // This can be present (sees tag) or not present (does not see tag)
            LimelightHelpers.Results result = limelight.getResults();
            // The skew of the tag represents how confident the camera is
            // If the result of the estimatedRobotPose exists,
            // and the skew of the tag is less than 3 degrees,
            // then we can confirm that the estimated position is realistic
            if (driver.getHID().getRightTriggerAxis() > 0 && !(result.botpose[0] == 0 && result.botpose[1] == 0) ) {
                swerve.getPoseEstimator().addVisionMeasurement( 
                    result.getBotPose2d_wpiBlue(),
                    Robot.currentTimestamp - limelight.getLatencyDiffSeconds());
            }
        }, limelight));

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX(),
            () -> !driver.getHID().getYButton(),
            () -> (driver.getHID().getYButton()
                && Robot.isRedAlliance())));
              
        configureButtonBindings();
        
        prepareNamedCommands();
        initializeArrays();
    }
    
    private void configureButtonBindings() {
        // configureDriverBindings(driver);
        configureOperatorBindings(driver);
        // configurePIDTunerBindings(driver);
        configureCalibrationBindings(operator);
    }
    
    private void configurePIDTunerBindings(PatriBoxController controller) {
        controller.povRight().onTrue(PIDTuner.incrementSubsystemCommand());
        controller.povLeft().onTrue(PIDTuner.decreaseSubsystemCommand());
        controller.rightBumper().onTrue(PIDTuner.PIDIncrementCommand());
        controller.leftBumper().onTrue(PIDTuner.PIDDecreaseCommand());
        controller.povUp().onTrue(PIDTuner.increaseCurrentPIDCommand(.001));
        controller.povDown().onTrue(PIDTuner.decreaseCurrentPIDCommand(.001));
        controller.a().onTrue(PIDTuner.logCommand());
        controller.x().onTrue(PIDTuner.multiplyPIDCommand(2));
        controller.b().onTrue(PIDTuner.multiplyPIDCommand(.5));
    }
  

    private void configureOperatorBindings(PatriBoxController controller) {
        controller.b().onTrue(shooterCalc.stopAllMotors().alongWith(pieceControl.stopAllMotors()));
        controller.povUp().toggleOnTrue(climb.povUpCommand(swerve::getPose));
        
        controller.povDown().onTrue(climb.toBottomCommand());

        // controller.povLeft().onTrue(elevator.toBottomCommand());

        // controller.povRight().onTrue(elevator.toTopCommand());

        controller.leftBumper()
            .onTrue(pieceControl.noteToShoot());

        controller.rightBumper()
            .onTrue(pieceControl.ejectNote());

        controller.a()
            .toggleOnTrue(shooterCalc.prepareSWDCommand(swerve::getPose, swerve::getRobotRelativeVelocity));

        controller.start().or(controller.back())
            .onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(1.332, 5.587, Rotation2d.fromDegrees(180)))));
        
        controller.rightStick()
            .toggleOnTrue(
                Commands.sequence(
                swerve.resetHDC(),
                swerve.getDriveCommand(
                    () -> {
                        return new ChassisSpeeds(
                            controller.getLeftY(),
                            controller.getLeftX(),
                            swerve.getAlignmentSpeeds(shooterCalc.calculateSWDRobotAngleToSpeaker(swerve.getPose(), swerve.getFieldRelativeVelocity())));
                    },
                    () -> true)));
        // controller.rightBumper()
        //     .and(controller.leftBumper().negate())
        //     .onTrue(pieceControl.noteToTarget(() -> true));

        // controller.leftTrigger(OIConstants.OPERATOR_DEADBAND)
        //     .and(intake.hasGamePieceTrigger().negate())
        //     .onTrue(pieceControl.intakeToClaw());

        // controller.leftTrigger()
        //     .onFalse(pieceControl.stopIntakeAndIndexer());

        // controller.rightTrigger(OIConstants.OPERATOR_DEADBAND)
        //     .onTrue(intake.outCommand());

        // controller.x().onTrue(intake.stop());

    }
    
    private void configureDriverBindings(PatriBoxController controller) {
        
        // Upon hitting start or back,
        // reset the orientation of the robot
        // to be facing away from the driver station
        // controller.start().or(controller.back()).onTrue(
        //     Commands.runOnce(() -> swerve.resetOdometry(
        //         new Pose2d(
        //             swerve.getPose().getTranslation(),
        //             Rotation2d.fromDegrees(
        //                 Robot.isRedAlliance()
        //                     ? 180
        //                     : 0))), 
        //         swerve));

        controller.b()
            .whileTrue(Commands.runOnce(swerve::getSetWheelsX));
        
        
        controller.leftStick()
            .toggleOnTrue(swerve.toggleSpeed());
        
        controller.a()
            .and(intake.hasGamePieceTrigger().negate())
            .onTrue(intake.inCommand());
        
        controller.y()
            .onTrue(intake.outCommand());
        
        controller.leftBumper()
            .toggleOnTrue(shooterCalc.prepareSWDCommand(swerve::getPose, swerve::getRobotRelativeVelocity));
        
        controller.x()
            .onTrue(intake.stop());
        
        controller.rightStick()
            .toggleOnTrue(
                Commands.sequence(
                swerve.resetHDC(),
                swerve.getDriveCommand(
                    () -> {
                        return new ChassisSpeeds(
                            controller.getLeftY(),
                            controller.getLeftX(),
                            swerve.getAlignmentSpeeds(shooterCalc.calculateSWDRobotAngleToSpeaker(swerve.getPose(), swerve.getFieldRelativeVelocity())));
                    },
                    () -> true)));

        controller.a().onTrue(shooterCalc.getNoteTrajectoryCommand(swerve::getPose, swerve::getRobotRelativeVelocity));
        controller.a().onFalse(shooterCalc.getNoteTrajectoryCommand(swerve::getPose, swerve::getRobotRelativeVelocity));
    }

    private void configureCalibrationBindings(PatriBoxController controller) {
        controller.leftBumper().onTrue(pieceControl.stopAllMotors().alongWith(shooterCalc.stopAllMotors()));
        controller.rightBumper().onTrue(calibrationControl.updateMotorsCommand());
        controller.rightTrigger().onTrue(pieceControl.shootWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity));

        controller.leftY().whileTrue(calibrationControl.incrementSpeeds(() -> (int) (controller.getLeftY() * 5)));
        controller.rightY().whileTrue(calibrationControl.incrementAngle(() -> -controller.getRightY()));

        controller.leftX().whileTrue(calibrationControl.incrementLeftSpeed(() -> (int) (controller.getLeftX() * 5)));
        controller.rightX().whileTrue(calibrationControl.incrementRightSpeed(() -> (int) (controller.getRightX() * 5)));

        controller.back().onTrue(calibrationControl.incrementDistance(-1));
        controller.start().onTrue(calibrationControl.incrementDistance(1));

        controller.a().onTrue(calibrationControl.logTriplet());

        controller.x().onTrue(calibrationControl.toggleLeftLock());
        controller.b().onTrue(calibrationControl.toggleRightLock());
        controller.y().onTrue(calibrationControl.togglePivotLock());

        controller.povLeft()
            .onTrue(pieceControl.noteToTrap());

        controller.povRight()
            .onTrue(pieceControl.ejectNote());

        controller.povDown()
            .onTrue(pieceControl.stopIntakeAndIndexer());
    }
    
    public Command getAutonomousCommand() {
        return Commands.none();
    }
    
    public void onDisabled() {
    }
    
    public void onEnabled() {
    }
    
    public void prepareNamedCommands() {
        // TODO: prepare to shoot while driving (w1 - c1)
        NamedCommands.registerCommand("intake", intake.inCommand());
        NamedCommands.registerCommand("shoot", pieceControl.noteToShoot());
        NamedCommands.registerCommand("placeAmp", pieceControl.noteToTarget(() -> true));
        NamedCommands.registerCommand("prepareShooterL", shooterCalc.prepareFireCommand(() -> true, () -> FieldConstants.L_POSE));
        NamedCommands.registerCommand("prepareShooterM", shooterCalc.prepareFireCommand(() -> true, () -> FieldConstants.M_POSE));
        NamedCommands.registerCommand("prepareShooterR", shooterCalc.prepareFireCommand(() -> true, () -> FieldConstants.R_POSE));
    }
    
    private void incinerateMotors() {
        Timer.delay(0.25);
        for (CANSparkBase neo : NeoMotorConstants.MOTOR_LIST) {
            neo.burnFlash();
            Timer.delay(0.005);
        }
        Timer.delay(0.25);
    }

    private void initializeArrays() {
        Pose3d initialShooterPose = new Pose3d(
                NTConstants.PIVOT_OFFSET_METERS.getX(),
                0,
                NTConstants.PIVOT_OFFSET_METERS.getY(),
            new Rotation3d()
        );

        components3d[0] = initialShooterPose;
        desiredComponents3d[0] = initialShooterPose;

        for (int i = 1; i < components3d.length; i++) {
            components3d[i] = new Pose3d();
            desiredComponents3d[i] = new Pose3d();
        }

        notePose3ds[0] = new Pose3d();
        for (int i = 1; i < notePose3ds.length; i++) {
            notePose3ds[i] = new Pose3d(FieldConstants.NOTE_TRANSLATIONS[i-1], new Rotation3d());
        }
    }
}

