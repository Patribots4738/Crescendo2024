package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PatriBoxController;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import monologue.Annotations.Log;
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
    
    @Log
    public static Pose3d[] components3d = new Pose3d[5];

    @Log
    public static Pose3d[] desiredComponents3d = new Pose3d[5];

    @Log
    public static Pose3d[] notePose3ds = new Pose3d[12];
    
    public RobotContainer() {
        
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);
        DriverStation.silenceJoystickConnectionWarning(true);
        
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
        
        pieceControl = new PieceControl(
                intake,
                triggerWheel,
                elevator,
                claw,
                shooterCalc);

        limelight.setDefaultCommand(Commands.run(() -> {
            // Create an "Optional" object that contains the estimated pose of the robot
            // This can be present (sees tag) or not present (does not see tag)
            LimelightHelpers.Results result = limelight.getResults();
            // The skew of the tag represents how confident the camera is
            // If the result of the estimatedRobotPose exists,
            // and the skew of the tag is less than 3 degrees,
            // then we can confirm that the estimated position is realistic
            if (result.valid) {
                swerve.getPoseEstimator().addVisionMeasurement( 
                    result.getBotPose2d(),
                    Robot.currentTimestamp - limelight.getLatencyDiffSeconds());
            }
        }, limelight));

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX(),
            () -> !driver.y().getAsBoolean(),
            () -> (driver.y().getAsBoolean()
                && Robot.isBlueAlliance())));
              
        configureButtonBindings();
        
        prepareNamedCommands();
        initializeArrays();
    }
    
    private void configureButtonBindings() {
        configureDriverBindings(driver);
        configureOperatorBindings(operator);
    }
    
    // TODO: uncomment these bindings (they are commented because we aren't testing them)
    private void configureOperatorBindings(PatriBoxController controller) {

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
            .onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(3,6.6, new Rotation2d()))));
        
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
        controller.start().or(controller.back()).onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(),
                    Rotation2d.fromDegrees(
                        Robot.isRedAlliance()
                            ? 0
                            : 180))), 
                swerve));

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
                        ;
                        return new ChassisSpeeds(
                            -controller.getLeftY(),
                            -controller.getLeftX(),
                            swerve.getAlignmentSpeeds(shooterCalc.calculateSWDRobotAngleToSpeaker(swerve.getPose(), swerve.getFieldRelativeVelocity())));
                    },
                    () -> true)));

        controller.a().onTrue(shooterCalc.getNoteTrajectoryCommand(swerve::getPose, swerve::getRobotRelativeVelocity));
        controller.a().onFalse(shooterCalc.getNoteTrajectoryCommand(swerve::getPose, swerve::getRobotRelativeVelocity));
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
        for (CANSparkBase neo : NeoMotorConstants.motors) {
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

