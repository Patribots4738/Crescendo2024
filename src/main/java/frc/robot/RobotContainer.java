package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.GameMode;
import frc.robot.commands.autonomous.ChoreoStorage;
import frc.robot.commands.autonomous.PathPlannerStorage;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.leds.LPI;
import frc.robot.commands.subsytemHelpers.AlignmentCmds;
import frc.robot.commands.subsytemHelpers.PieceControl;
import frc.robot.commands.subsytemHelpers.ShooterCmds;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Trapper;
import frc.robot.subsystems.misc.leds.LedStrip;
import frc.robot.subsystems.misc.limelight.Limelight;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.calc.LimelightHelpers;
import frc.robot.util.constants.Constants.AutoConstants;
import frc.robot.util.constants.Constants.FieldConstants;
import frc.robot.util.constants.Constants.NTConstants;
import frc.robot.util.constants.Constants.NeoMotorConstants;
import frc.robot.util.constants.Constants.OIConstants;
import frc.robot.util.mod.PatriBoxController;
import frc.robot.util.motors.Neo;
import frc.robot.util.testing.CalibrationControl;
import frc.robot.util.testing.HDCTuner;
import frc.robot.util.testing.PIDNotConstants;
import frc.robot.util.testing.PIDTunerCommands;
import monologue.Annotations.Log;
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
    private Trapper trapper;
    private Elevator elevator;
    private ShooterCmds shooterCmds;
    private PieceControl pieceControl;
    private ChoreoStorage choreoPathStorage;
    private PathPlannerStorage pathPlannerStorage;
    private CalibrationControl calibrationControl;
    private PIDTunerCommands PIDTuner;
    private AlignmentCmds alignmentCmds;

    public static HDCTuner HDCTuner;
    
    @Log
    public static Pose3d[] components3d = new Pose3d[5];

    @Log
    public static Pose3d[] desiredComponents3d = new Pose3d[5];

    @Log
    public static Pose3d[] notePose3ds = new Pose3d[12];

    @Log
    private boolean freshCode = true;

    @Log
    public static Field2d field2d = new Field2d();
    
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
        trapper = new Trapper();
        
        pivot = new Pivot();

        HDCTuner = new HDCTuner(
            AutoConstants.HDC.getXController(),
            AutoConstants.HDC.getThetaController());

        incinerateMotors();
        
        shooterCmds = new ShooterCmds(shooter, pivot);
        
        PIDTuner = new PIDTunerCommands(new PIDNotConstants[] {
            swerve.getDrivingPidNotConstants(),
            swerve.getTurningPidNotConstants()
        });

        alignmentCmds = new AlignmentCmds(swerve, climb, shooterCmds);

        pieceControl = new PieceControl(
            intake,
            triggerWheel,
            elevator,
            trapper,
            shooterCmds);

        calibrationControl = new CalibrationControl(shooterCmds);

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

        pathPlannerStorage = new PathPlannerStorage(driver.y().negate());
        initializeArrays();
        prepareNamedCommands();
        // choreoPathStorage = new ChoreoStorage(driver.y());
        // setupChoreoChooser();
        pathPlannerStorage.configureAutoChooser();
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        if (FieldConstants.IS_SIMULATION) {
            configureSimulationBindings(driver);
        }
        configureDriverBindings(driver);
        configureOperatorBindings(operator);
        configureTestBindings();

        new Trigger(Robot::isRedAlliance)
            .onTrue(pathPlannerStorage.updatePathViewerCommand())
            .onFalse(pathPlannerStorage.updatePathViewerCommand());
    }

    private void configureTestBindings() {
        // Warning: these buttons are not on the default loop!
        // See https://docs.wpilib.org/en/stable/docs/software/convenience-features/event-based.html
        // for more information 
        configureHDCTuner(driver);
        configureCalibrationBindings(operator);
    }
    
    private void configureOperatorBindings(PatriBoxController controller) {
        controller.povUp()
            .onTrue(elevator.toTopCommand());
        
        controller.povDown()
            .onTrue(elevator.toBottomCommand());

        controller.leftBumper()
            .onTrue(pieceControl.toggleIn());

        controller.rightBumper()
            .onTrue(pieceControl.toggleOut());

        controller.x()
            .onTrue(pieceControl.setShooterModeCommand(true));

        controller.b()
            .onTrue(pieceControl.setShooterModeCommand(false));
    }
    
    private void configureDriverBindings(PatriBoxController controller) {
        
        // Upon hitting start or back,
        // reset the orientation of the robot
        // to be facing AWAY FROM the driver station
        controller.back().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    1.31,
                    5.53, 
                    Rotation2d.fromDegrees(
                        Robot.isRedAlliance()
                            ? 0
                            : 180))), 
                swerve));

        // Upon hitting start button
        // reset the orientation of the robot
        // to be facing TOWARDS the driver station
        // TODO: for testing reset odometry to speaker
        controller.start().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(),
                    Rotation2d.fromDegrees(
                        Robot.isRedAlliance()
                            ? 180
                            : 0))), 
                swerve));

        controller.povUp()
            .toggleOnTrue(climb.povUpCommand(swerve::getPose));
        
        controller.povDown().onTrue(climb.toBottomCommand());
        
        controller.a().whileTrue(
            Commands.sequence(
                swerve.resetHDC(),
                Commands.either(
                    alignmentCmds.trapAlignmentCommand(() -> controller.getLeftY()), 
                    alignmentCmds.ampAlignmentCommand(() -> controller.getLeftX()), 
                    climb::hooksUp)));
        
        
        controller.rightTrigger()
            .onTrue(pieceControl.noteToTarget(swerve::getPose, swerve::getRobotRelativeVelocity));

        controller.rightStick()
            .toggleOnTrue(
                Commands.sequence(
                    swerve.resetHDC(),
                    Commands.either(
                        alignmentCmds.sourceRotationalAlignment(controller::getLeftX, controller::getLeftY),
                        alignmentCmds.wingRotationalAlignment(controller::getLeftX, controller::getLeftY),
                        alignmentCmds.alignmentCalc::onOppositeSide)));

        controller.b()
            .onTrue(pieceControl.stopAllMotors());

        controller.x()
            .toggleOnTrue(shooterCmds.prepareSWDCommand(swerve::getPose, swerve::getRobotRelativeVelocity));

        controller.leftBumper()
            .onTrue(pieceControl.toggleIn());

        controller.rightBumper()
            .onTrue(pieceControl.toggleOut());
    }
    
    private void configureSimulationBindings(PatriBoxController controller) {
        controller.rightTrigger().onTrue(shooterCmds.getNoteTrajectoryCommand(swerve::getPose, swerve::getRobotRelativeVelocity));
        controller.rightTrigger().onFalse(shooterCmds.getNoteTrajectoryCommand(swerve::getPose, swerve::getRobotRelativeVelocity));
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
        controller.leftBumper(testButtonBindingLoop).onTrue(pieceControl.stopAllMotors());
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
        return driver.getYButton() ? choreoChooser.getSelected() : pathPlannerStorage.getSelectedAuto();
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
        swerve.stopDriving();
        pieceControl.stopAllMotors().schedule();
        pathPlannerStorage.updatePathViewerCommand().schedule();
    }
    
    public void onEnabled() {
        if (Robot.gameMode == GameMode.TELEOP) {
            new LPI(ledStrip, swerve::getPose, operator, swerve::setDesiredPose).schedule();
            pathPlannerStorage.updatePathViewerCommand().schedule();
        }
        this.freshCode = false;
    }

    public void onTest() {
        CommandScheduler.getInstance().setActiveButtonLoop(testButtonBindingLoop);
    }
    
    public void prepareNamedCommands() {
        // TODO: prepare to shoot while driving (w1 - c1)
        NamedCommands.registerCommand("Intake", pieceControl.intakeAuto());
        NamedCommands.registerCommand("StopIntake", pieceControl.stopIntakeAndIndexer());
        NamedCommands.registerCommand("StopAll", pieceControl.stopAllMotors());
        NamedCommands.registerCommand("PrepareShooter", shooterCmds.prepareFireCommandAuto(swerve::getPose));
        NamedCommands.registerCommand("Shoot", pieceControl.noteToShoot());
        NamedCommands.registerCommand("ShootWhenReady", pieceControl.shootWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity));
        NamedCommands.registerCommand("PlaceAmp", pieceControl.elevatorPlacementCommand());
        NamedCommands.registerCommand("PrepareShooterL", shooterCmds.prepareFireCommand(() -> FieldConstants.L_POSE));
        NamedCommands.registerCommand("PrepareShooterM", shooterCmds.prepareFireCommand(() -> FieldConstants.M_POSE));
        NamedCommands.registerCommand("PrepareShooterR", shooterCmds.prepareFireCommand(() -> FieldConstants.R_POSE));
        NamedCommands.registerCommand("PrepareShooterW3", shooterCmds.prepareFireCommand(() -> FieldConstants.W3_POSE));
        NamedCommands.registerCommand("PrepareShooter", shooterCmds.prepareFireCommand(pathPlannerStorage::getNextShotTranslation));
        NamedCommands.registerCommand("PrepareSWD", shooterCmds.prepareSWDCommand(swerve::getPose, swerve::getRobotRelativeVelocity));
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
        for (Neo neo : NeoMotorConstants.MOTOR_LIST) {
            neo.burnFlash();
        }
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