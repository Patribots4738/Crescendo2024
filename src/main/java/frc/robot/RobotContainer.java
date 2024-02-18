package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveHDC;
import frc.robot.commands.PieceControl;
import frc.robot.commands.ShooterCalc;
import frc.robot.commands.autonomous.ChoreoStorage;
import frc.robot.commands.autonomous.PathPlannerStorage;
import frc.robot.commands.leds.LPI;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.CalibrationControl;
import frc.robot.util.HDCTuner;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PatriBoxController;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.FieldConstants.GameMode;
import monologue.Annotations.Log;
import frc.robot.util.PIDNotConstants;
import frc.robot.util.PIDTunerCommands;
import monologue.Logged;

public class RobotContainer implements Logged {

    private EventLoop testButtonBindingLoop = new EventLoop();
    
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
    private ChoreoStorage choreoPathStorage;
    private PathPlannerStorage pathPlannerStorage;
    private CalibrationControl calibrationControl;
    private PIDTunerCommands PIDTuner;

    public static HDCTuner HDCTuner;
    
    @Log
    public static Pose3d[] components3d = new Pose3d[5];

    @Log
    public static Pose3d[] desiredComponents3d = new Pose3d[5];

    @Log
    public static Pose3d[] notePose3ds = new Pose3d[12];

    @Log
    private boolean freshCode = true;
    
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

        HDCTuner = new HDCTuner(
            AutoConstants.HDC.getXController(),
            AutoConstants.HDC.getThetaController());

        incinerateMotors();
        
        shooterCalc = new ShooterCalc(shooter, pivot);
        
        PIDTuner = new PIDTunerCommands(new PIDNotConstants[] {
            swerve.getDrivingPidNotConstants(),
            swerve.getTurningPidNotConstants()
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
            if ( // check validity
                ((driver.getHID().getLeftTriggerAxis() > 0 && !(result.botpose[0] == 0 && result.botpose[1] == 0) )
                // check if good tag
                && (LimelightHelpers.getTA("limelight") >= 0.3 
                    || result.targets_Fiducials.length > 1 && LimelightHelpers.getTA("limelight") > 0.4))
                && limelight.getRobotPoseTargetSpace().getTranslation().getNorm() < 3.25
            ) {
                Pose2d estimatedRobotPose = result.getBotPose2d_wpiBlue();
                if (Double.isNaN(estimatedRobotPose.getX()) 
                    || Double.isNaN(estimatedRobotPose.getY()) 
                    || Double.isNaN(estimatedRobotPose.getRotation().getRadians())) {
                    return;
                }
                swerve.getPoseEstimator().addVisionMeasurement( 
                    estimatedRobotPose,
                    Robot.currentTimestamp - limelight.getLatencyDiffSeconds());
            }
        }, limelight));

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX(),
            () -> !driver.getYButton(),
            () -> (!driver.getYButton()
                && Robot.isRedAlliance()))); 
        
        configureButtonBindings();
        initializeArrays();
        
        pathPlannerStorage = new PathPlannerStorage(driver.y());
        prepareNamedCommands();
        // choreoPathStorage = new ChoreoStorage(driver.y());
        // setupChoreoChooser();
        pathPlannerStorage.configureAutoChooser();
    }
    
    private void configureButtonBindings() {
        configureDriverBindings(driver);
        configureOperatorBindings(operator);
        configureTestBindings();
    }

    private void configureTestBindings() {
        // Warning: these buttons are not on the default loop!
        // See https://docs.wpilib.org/en/stable/docs/software/convenience-features/event-based.html
        // for more information 
        configureHDCTuner(driver);
        configureCalibrationBindings(operator);
    }
    
    private void configureOperatorBindings(PatriBoxController controller) {
        controller.start().or(controller.back())
            .onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(FieldConstants.FIELD_WIDTH_METERS - 1.332, 5.587, Rotation2d.fromDegrees(180)))));

        controller.leftBumper()
            .onTrue(pieceControl.noteToTrap());


        controller.rightBumper()
            .onTrue(pieceControl.ejectNote());

        controller.b()
            .onTrue(pieceControl.stopIntakeAndIndexer());
    }
    
    private void configureDriverBindings(PatriBoxController controller) {
        controller.b()
            .onTrue(shooterCalc.stopAllMotors()
                .alongWith(pieceControl.stopAllMotors()));
        
        controller.povUp()
            .toggleOnTrue(climb.povUpCommand(swerve::getPose));

        controller.povDown()
            .onTrue(climb.toBottomCommand());

        controller.leftBumper()
            .onTrue(pieceControl.noteToShoot());

        controller.rightBumper()
            .onTrue(pieceControl.ejectNote());

        controller.x()
            .toggleOnTrue(shooterCalc.prepareFireCommand(swerve::getPose));
        
        controller.back()
            .onTrue(pieceControl.sourceShooterIntake(controller.start()));

        controller.a()
            .whileTrue(
                Commands.sequence(
                    swerve.resetHDC(),
                    swerve.ampAlignmentCommand(() -> driver.getLeftX())));

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

        // Upon hitting start button
        // reset the orientation of the robot
        // to be facing AWAY FROM the driver station
        controller.back().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(),
                    Rotation2d.fromDegrees(
                        Robot.isRedAlliance()
                            ? 0
                            : 180))), 
                swerve));

        // Upon hitting start button
        // reset the orientation of the robot
        // to be facing TOWARDS the driver station
        controller.start().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(),
                    Rotation2d.fromDegrees(
                        Robot.isRedAlliance()
                            ? 180
                            : 0))), 
                swerve));

        controller.b()
            .whileTrue(Commands.run(swerve::getSetWheelsX));
        
        controller.leftStick()
            .toggleOnTrue(swerve.toggleSpeed());
        
        // controller.leftBumper()
        //     .and(intake.hasGamePieceTrigger().negate())
        //     .onTrue(intake.inCommand());$
        
        controller.rightTrigger()
            .onTrue(pieceControl.shootWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity));
    }
    
    private void configurePIDTunerBindings(PatriBoxController controller) {
        controller.pov(0, 270, testButtonBindingLoop)
            .onTrue(PIDTuner.incrementSubsystemCommand());

        controller.pov(0, 90, testButtonBindingLoop)
            .onTrue(PIDTuner.decreaseSubsystemCommand());
            
        controller.pov(0, 0, testButtonBindingLoop)
            .onTrue(PIDTuner.increaseCurrentPIDCommand(.1));
            
        controller.pov(0, 180, testButtonBindingLoop)
            .onTrue(PIDTuner.increaseCurrentPIDCommand(-.1));
            
        controller.rightBumper(testButtonBindingLoop)
            .onTrue(PIDTuner.PIDIncrementCommand());
            
        controller.leftBumper(testButtonBindingLoop)
            .onTrue(PIDTuner.PIDDecreaseCommand());
            
        controller.a(testButtonBindingLoop)
            .onTrue(PIDTuner.logCommand());
            
        controller.x(testButtonBindingLoop)
            .onTrue(PIDTuner.multiplyPIDCommand(2));
            
        controller.b(testButtonBindingLoop)
            .onTrue(PIDTuner.multiplyPIDCommand(.5));
            
    }
    
    private void configureCalibrationBindings(PatriBoxController controller) {
        controller.leftBumper(testButtonBindingLoop).onTrue(pieceControl.stopAllMotors().alongWith(shooterCalc.stopAllMotors()));
        controller.rightBumper(testButtonBindingLoop).onTrue(calibrationControl.updateMotorsCommand());
        controller.rightTrigger(0.5, testButtonBindingLoop).onTrue(pieceControl.shootWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity));

        controller.leftY(0.3, testButtonBindingLoop).whileTrue(calibrationControl.incrementSpeeds(() -> (int) (controller.getLeftY() * 5)));
        controller.rightY(0.3, testButtonBindingLoop).whileTrue(calibrationControl.incrementAngle(() -> -controller.getRightY()));

        controller.leftX(0.3, testButtonBindingLoop).whileTrue(calibrationControl.incrementLeftSpeed(() -> (int) (controller.getLeftX() * 5)));
        controller.rightX(0.3, testButtonBindingLoop).whileTrue(calibrationControl.incrementRightSpeed(() -> (int) (controller.getRightX() * 5)));

        controller.back(testButtonBindingLoop).onTrue(calibrationControl.incrementDistance(-1));
        controller.start(testButtonBindingLoop).onTrue(calibrationControl.incrementDistance(1));

        controller.a(testButtonBindingLoop).onTrue(calibrationControl.logTriplet());

        controller.x(testButtonBindingLoop).onTrue(calibrationControl.toggleLeftLock());
        controller.b(testButtonBindingLoop).onTrue(calibrationControl.toggleRightLock());
        controller.y(testButtonBindingLoop).onTrue(calibrationControl.togglePivotLock());

        controller.pov(0, 270, testButtonBindingLoop)
            .onTrue(pieceControl.noteToTrap());

        controller.pov(0, 90, testButtonBindingLoop)
            .onTrue(pieceControl.ejectNote());

        controller.pov(0, 0, testButtonBindingLoop)
            .onTrue(pieceControl.stopIntakeAndIndexer());

        controller.pov(0, 180, testButtonBindingLoop)
            .onTrue(calibrationControl.copyCalcTriplet());
    }
    
    private void configureHDCTuner(PatriBoxController controller) {
        controller.pov(0, 270, testButtonBindingLoop)
            .onTrue(HDCTuner.controllerDecrementCommand());
        controller.pov(0, 90, testButtonBindingLoop)
            .onTrue(HDCTuner.controllerIncrementCommand());
        controller.pov(0, 0, testButtonBindingLoop)
            .onTrue(HDCTuner.increaseCurrentConstantCommand(.1));
        controller.pov(0, 180, testButtonBindingLoop)
            .onTrue(HDCTuner.increaseCurrentConstantCommand(-.1));
        controller.rightBumper(testButtonBindingLoop)
            .onTrue(HDCTuner.constantIncrementCommand());
        controller.leftBumper(testButtonBindingLoop)
            .onTrue(HDCTuner.constantDecrementCommand());
        controller.a(testButtonBindingLoop)
            .onTrue(HDCTuner.logCommand());
        controller.x(testButtonBindingLoop)
            .onTrue(HDCTuner.multiplyPIDCommand(2));
        controller.b(testButtonBindingLoop)
            .onTrue(HDCTuner.multiplyPIDCommand(.5));
    }

    public Command getAutonomousCommand() {
        return driver.y().getAsBoolean() ? choreoChooser.getSelected() : pathPlannerStorage.getSelectedAuto();
    }

    @Log.NT
    public static SendableChooser<Command> choreoChooser = new SendableChooser<>();
    // PathPlannerPath starting = PathPlannerPath.fromChoreoTrajectory("S W3-1S C1");
    private void setupChoreoChooser() {
        // // TODO: Autos currently start at C1-5, we need to integrate the other paths
        // // with the center line schenanigans to make full autos
        // choreoChooser.setDefaultOption("Do Nothing", Commands.none());
        // choreoChooser.addOption("W3-1 C1-5", 
        //     swerve.resetOdometryCommand(
        //         () -> starting.getPreviewStartingHolonomicPose()
        //             .plus(new Transform2d(
        //                     new Translation2d(), 
        //                     Rotation2d.fromDegrees(180))))
        //     .andThen(
        //         AutoBuilder.followPath(starting)
        //         .andThen(choreoPathStorage.generateCenterLineComplete(1, 5, false))));
    }

    public void onDisabled() {
        swerve.stopMotors();
    }
    
    public void onEnabled() {
        if (FieldConstants.GAME_MODE == GameMode.TELEOP)
            new LPI(ledStrip, swerve::getPose, operator, swerve::setDesiredPose).schedule();
        this.freshCode = false;
    }

    public void onTest() {
        CommandScheduler.getInstance().setActiveButtonLoop(testButtonBindingLoop);
    }
    
    public void prepareNamedCommands() {
        // TODO: prepare to shoot while driving (w1 - c1)
        NamedCommands.registerCommand("Intake", pieceControl.noteToShoot());
        NamedCommands.registerCommand("StopIntake", pieceControl.stopIntakeAndIndexer());
        NamedCommands.registerCommand("PrepareShooter", shooterCalc.prepareFireCommand(swerve::getPose));
        NamedCommands.registerCommand("Shoot", pieceControl.noteToShoot());
        NamedCommands.registerCommand("ShootWhenReady", pieceControl.shootWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity));
        NamedCommands.registerCommand("PlaceAmp", pieceControl.noteToTarget());
        NamedCommands.registerCommand("PrepareShooterL", shooterCalc.prepareFireCommand(() -> FieldConstants.L_POSE));
        NamedCommands.registerCommand("PrepareShooterM", shooterCalc.prepareFireCommand(() -> FieldConstants.M_POSE));
        NamedCommands.registerCommand("PrepareShooterR", shooterCalc.prepareFireCommand(() -> FieldConstants.R_POSE));
        for (int i = 1; i <= FieldConstants.CENTER_NOTE_COUNT; i++) {
            for (int j = 1; j <= FieldConstants.CENTER_NOTE_COUNT; j++) {
                if (i == j) {
                    continue;
                }
                NamedCommands.registerCommand("C" + i + "toC" + j, pathPlannerStorage.generateCenterLogic(i, j));
            }
        }
    }
    
    private void incinerateMotors() {
        Timer.delay(1);
        for (CANSparkBase neo : NeoMotorConstants.MOTOR_LIST) {
            neo.burnFlash();
            Timer.delay(0.05);
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