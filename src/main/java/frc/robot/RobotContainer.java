package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.GameMode;
import frc.robot.commands.drive.AlignmentCmds;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.commands.logging.NT;
import frc.robot.commands.logging.NTPIDTuner;
import frc.robot.commands.managers.CalibrationControl;
import frc.robot.commands.managers.HDCTuner;
import frc.robot.commands.managers.PieceControl;
import frc.robot.commands.managers.ShooterCmds;
import frc.robot.leds.Strips.LedStrip;
import frc.robot.leds.Commands.LPI;
import frc.robot.subsystems.*;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.CameraConstants;
import frc.robot.util.Constants.ColorSensorConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.calc.LimelightMapping;
import frc.robot.util.calc.ShooterCalc;
import frc.robot.util.custom.PatriBoxController;
import frc.robot.util.custom.ActiveConditionalCommand;
import frc.robot.util.rev.Neo;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

    private PowerDistribution pdh;

    //* We add a event loop here so we can use different bindings in a different mode
    //* of the code (there are too many binding to fit on the controller)
    private EventLoop testButtonBindingLoop = new EventLoop();
    
    private final PatriBoxController driver;
    private final PatriBoxController operator;

    @IgnoreLogged
    private Swerve swerve;
    @IgnoreLogged
    private final Intake intake;
    @IgnoreLogged
    private Limelight limelight3;
    @IgnoreLogged
    private Limelight limelight2;
    @IgnoreLogged
    private LimelightMapping limelightMapper;
    @IgnoreLogged
    private final Climb climb;
    @IgnoreLogged
    private Pivot pivot;
    @IgnoreLogged
    private Shooter shooter;
    @IgnoreLogged
    private Elevator elevator;
    
    @IgnoreLogged
    private CalibrationControl calibrationControl;
    @IgnoreLogged
    private PathPlannerStorage pathPlannerStorage;
    @IgnoreLogged
    private ShooterCalc shooterCalc;
    @IgnoreLogged
    public static HDCTuner HDCTuner;

    private final LedStrip ledStrip;
    @IgnoreLogged
    private Indexer indexer;
    @IgnoreLogged
    private Ampper ampper;
    private ShooterCmds shooterCmds;
    @IgnoreLogged
    private ColorSensor colorSensor = new ColorSensor(ColorSensorConstants.I2C_PORT);

    @IgnoreLogged
    private PieceControl pieceControl;
    private AlignmentCmds alignmentCmds;
    private final BooleanSupplier robotRelativeSupplier;
    
    @Log
    public static Pose3d[] components3d = new Pose3d[5];
    @Log
    public static Pose3d[] desiredComponents3d = new Pose3d[5];
    @Log
    public static Pose3d[] notePose3ds = new Pose3d[12];
    @Log
    public static Pose3d[] highNotePose3ds = new Pose3d[12];
    @Log
    private boolean freshCode = true;
    @Log
    public static Field2d field2d = new Field2d();
    @Log
    public static Pose2d robotPose2d = new Pose2d();
    @Log
    public static Transform2d visionErrorPose = new Transform2d();
    @Log
    public static double distanceToSpeakerMeters = 0;
    @Log
    public static Pose3d robotPose3d = new Pose3d();
    @Log
    public static SwerveModuleState[] swerveMeasuredStates;
    @Log
    public static SwerveModuleState[] swerveDesiredStates;
    @Log
    public static double gameModeStart = 0;
    @Log
    public static boolean hasPiece = true;
    @Log
    public static boolean enableVision = true;
    
    public RobotContainer() {
        
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);
        
        pdh = new PowerDistribution(30, ModuleType.kRev);
        pdh.setSwitchableChannel(false); 

        intake = new Intake();
        climb = new Climb();
        swerve = new Swerve();
        if (CameraConstants.FIELD_CALIBRATION_MODE) {
            limelight3 = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "not-limelight-three", 0);
            limelight2 = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "not-limelight-two", 1);
            limelightMapper = new LimelightMapping(swerve.getPoseEstimator(), swerve::getPose, "limelight-three");
            
            // limelightMapper.ManuallyAddPose(12, new Pose3d(11.173, 4.096, 1.443, new Rotation3d(new Quaternion(0.003, 0.032, -0.025, -0.999))));
            // limelightMapper.ManuallyAddPose(2, new Pose3d(16.787, 5.036, 1.562, new Rotation3d(new Quaternion(.00099, -0.019, -0.017, -1.000))));
            // limelightMapper.ManuallyAddPose(3, new Pose3d(16.776, 5.599, 1.562, new Rotation3d(new Quaternion(0.009, 0.016, -0.003, -1.000))));
            // limelightMapper.printJSON();
        } else {
            limelight3 = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "limelight-three", 0);
            limelight2 = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "limelight-two", 1);
            limelight2.disableLEDS();
            limelight3.disableLEDS();
        }


        ledStrip = new LedStrip(swerve::getPose);
        indexer = new Indexer();

        shooter = new Shooter();
        elevator = new Elevator();
        ampper = new Ampper();
        
        pivot = new Pivot();

        HDCTuner = new HDCTuner(
            AutoConstants.HDC.getXController(),
            AutoConstants.HDC.getThetaController());

        Neo.incinerateMotors();
        new NTPIDTuner().schedule();
        new NT(NTConstants.WAIT_TIMES);

        shooterCalc = new ShooterCalc(shooter, pivot);
        shooterCmds = new ShooterCmds(shooter, pivot, shooterCalc);

        alignmentCmds = new AlignmentCmds(swerve, climb, shooterCmds);

        pieceControl = new PieceControl(
            intake,
            indexer,
            elevator,
            ampper,
            shooterCmds,
            colorSensor);

        calibrationControl = new CalibrationControl(shooterCmds);

        robotRelativeSupplier = () -> !driver.getYButton();

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX()/1.6,
            robotRelativeSupplier,
            () -> (robotRelativeSupplier.getAsBoolean() && Robot.isRedAlliance())
        ));

        shooter.setDefaultCommand(
            pieceControl.getAutomaticShooterSpeeds(swerve::getPose)
        );
        
        pathPlannerStorage = new PathPlannerStorage(driver.y().negate(), colorSensor::hasNote, swerve, limelight2);
        initializeComponents();
        prepareNamedCommands();

        pathPlannerStorage.configureAutoChooser();
        pathPlannerStorage.getAutoChooser().addOption("WheelRadiusCharacterization",
            disableVision()
            .andThen(
                swerve.setWheelsOCommand()
                .andThen(new WheelRadiusCharacterization(swerve)))
            .andThen(enableVision()));
        
        configureButtonBindings();
        configureLoggingPaths();

        pdh.setSwitchableChannel(false);

    }
    
    private void configureButtonBindings() {

        if (CameraConstants.FIELD_CALIBRATION_MODE) {
            configureFieldCalibrationBindings(driver);
            return;
        }

        if (OIConstants.SINGLE_DRIVER_MODE) {
            configureSoloDiverBindings(driver);
        } else {
            configureDriverBindings(driver);
            configureOperatorBindings(operator);
        }
        

        configureTimedEvents();
        configureTestBindings();
    }

    private void configureTestBindings() {
        // Warning: these buttons are not on the default loop!
        // See https://docs.wpilib.org/en/stable/docs/software/convenience-features/event-based.html
        // for more information 
        // configureHDCBindings(driver);
        configureCalibrationBindings(driver);
    }
    
    private void configureTimedEvents() {
      
        // In the last couple seconds of the match,
        // "hail mary" the note to get a last second score just in case
        new Trigger(() -> 
               Robot.currentTimestamp - gameModeStart >= 134.2 
            && Robot.gameMode == GameMode.TELEOP 
            && RobotController.getBatteryVoltage() > 10 
            && DriverStation.isFMSAttached()
        ).onTrue(pieceControl.coastIntakeAndIndexer()
            .andThen(pieceControl.noteToShoot(swerve::getPose, swerve::getRobotRelativeVelocity))
            .andThen(Commands.waitSeconds(5))
            .andThen(pieceControl.brakeIntakeAndIndexer()));

        // The switchable channel on our bot is a bright white lamp
        // This lamp turns on when the robot knows where it is 
        // and is confident it will make the shot
        // if shootWhenReady is called at while this condition is true
        new Trigger(() -> 
            Robot.gameMode == GameMode.TELEOP
            && shooter.getAverageSpeed() > 2500
            && swerve.getPose().getX() > FieldConstants.CENTERLINE_X ^ Robot.isBlueAlliance()
            && limelight3.getPose2d().getTranslation().getDistance(swerve.getPose().getTranslation()) < Units.inchesToMeters(4))
        .onTrue(Commands.runOnce(() -> pdh.setSwitchableChannel(true)))
        .onFalse(Commands.runOnce(() -> pdh.setSwitchableChannel(false)));
        
        // When our alliance changes, reflect that in the path previewer
        new Trigger(Robot::isRedAlliance)
            .onTrue(pathPlannerStorage.updatePathViewerCommand())
            .onFalse(pathPlannerStorage.updatePathViewerCommand());
        
        new Trigger(swerve::isAlignedToAmp).or(shooterCalc.readyToShootSupplier())
            .onTrue(driver.setRumble(() -> 0.5))
            .onFalse(driver.setRumble(() -> 0));
        
        // Make the controllers pulse just like the LEDs do
        // The reason we are checking bumpers
        // is so this doesn't happen on pieceControl::moveNote
        new Trigger(
            () -> colorSensor.hasNote() 
                && (
                    // All of these buttons command intake
                    // Whether that be source or not
                    driver.getLeftTrigger() 
                    || driver.getXButton() 
                    || operator.getLeftBumper() 
                    || operator.getStartButton() 
                    || operator.getBackButton()
                )
            ).onTrue(
                Commands.race(
                    // I'll be honest, I was just messing around with desmos
                    // and this provides a sort of on/off rumble rather than it being continuous
                    // You could say that it makes this rumble correlate to piece pickup since its distinct
                    Commands.run(() -> {
                        driver.setRumble(Math.cos(2*Math.PI*Robot.currentTimestamp*4)/2.0);
                    }),
                    Commands.waitSeconds(1)
                ).andThen(driver.setRumble(() -> 0))
                .alongWith(limelight3.blinkLeds(() -> 1))
                .alongWith(limelight2.blinkLeds(() -> 1))
            );

        // Have the controllers pulse when the match is about to end to signal the drivers
        // Pulses get more extreme as the clock approaches 0
        // Starts when there are five seconds left in the match
        new Trigger(() -> Robot.currentTimestamp - gameModeStart >= 130 && Robot.currentTimestamp - gameModeStart < 135 && Robot.gameMode == GameMode.TELEOP)
            .whileTrue(
                Commands.run(
                    () -> {
                        double rumbleSpeed =
                            (Math.cos(2*Math.PI*(135 - Robot.currentTimestamp - gameModeStart) + Math.PI)
                            /
                            ((135 - Robot.currentTimestamp - gameModeStart)*2.0)); 
                        driver.setRumble(rumbleSpeed);
                    }
                )
            ).onFalse(
                driver.setRumble(() -> 0)
            );
    }
    
    private void configureFieldCalibrationBindings(PatriBoxController controller) {
        driver.povLeft()
            .onTrue(
                limelightMapper.incrementCalibrationPose(false)
                .andThen(swerve.resetPositionCommand(limelightMapper::getCurrentCalibrationPosition))
            );

        driver.povRight()
            .onTrue(
                limelightMapper.incrementCalibrationPose(true)
                .andThen(swerve.resetPositionCommand(limelightMapper::getCurrentCalibrationPosition))
            );

        driver.povUp()
            .onTrue(
                swerve.resetOdometryCommand(
                    () -> new Pose2d(
                        swerve.getPose().getTranslation(), 
                        swerve.getPose().getRotation().plus(Rotation2d.fromDegrees(45)))));

        driver.povDown()
            .onTrue(
                swerve.resetOdometryCommand(
                    () -> new Pose2d(
                        swerve.getPose().getTranslation(), 
                        swerve.getPose().getRotation().minus(Rotation2d.fromDegrees(45)))));

        driver.a()
            .onTrue(Commands.runOnce(() -> limelightMapper.takeSnapshot()));

        driver.leftBumper().and(driver.rightBumper())
            .onTrue(limelightMapper.printJSONCommand());

        driver.leftStick()
            .whileTrue(limelightMapper.updatePoseEstimatorCommand());
    }

    private void configureCommonDriverBindings(PatriBoxController controller) {
        // Upon hitting start or back,
        // reset the orientation of the robot
        // to be facing AWAY FROM the driver station
        controller.back()
            .onTrue(Commands.runOnce(() -> 
                swerve.resetOdometry(FieldConstants.GET_SUBWOOFER_POSITION()), swerve
            ));
        
        // Upon hitting start button
        // reset the orientation of the robot
        // to be facing TOWARDS the driver station
        // TODO: for testing reset odometry to speaker
        controller.start()
            .onTrue(Commands.runOnce(() -> 
                swerve.resetOdometry(FieldConstants.GET_SUBWOOFER_POSITION().plus(new Transform2d(0,0, Rotation2d.fromDegrees(180)))), swerve
            ));


        // Speaker / Source / Chain rotational alignment
        controller.rightStick()
            .toggleOnTrue(
                Commands.sequence(
                    elevator.toBottomCommand(),
                    swerve.resetHDCCommand(),
                    limelight3.setLEDState(() -> true),
                    new ActiveConditionalCommand(
                        // This runs SWD on heading control 
                        // and shooting-while-still on shooter
                        alignmentCmds.wingRotationalAlignment(controller::getLeftX, controller::getLeftY, robotRelativeSupplier),
                        alignmentCmds.preparePassCommand(controller::getLeftX, controller::getLeftY, robotRelativeSupplier),
                        alignmentCmds.alignmentCalc::closeToSpeaker)
                    ).finallyDo(
                        () -> 
                            limelight3.setLEDState(() -> false)
                            .andThen(pivot.setAngleCommand(60)
                                .alongWith(shooterCmds.stopShooter())
                                .withInterruptBehavior(InterruptionBehavior.kCancelSelf))
                            .schedule()
                        )
                );


        // Climbing controls
        controller.povUp()
            .onTrue(climb.toTopCommand());

        controller.povDown()
            .onTrue(climb.toBottomCommand().alongWith(pivot.setAngleCommand(0)));
        
        // Note to target will either place amp or shoot,
        // depending on if the elevator is up or not
        controller.rightTrigger()
            .onTrue(pieceControl.noteToTarget(swerve::getPose, swerve::getRobotRelativeVelocity, swerve::atHDCAngle, () -> controller.getLeftBumper())
                .alongWith(driver.setRumble(() -> 0.5, 0.3))
            .andThen(Commands.runOnce(() -> RobotContainer.hasPiece = false)));
        

        // Intake controls
        // The warning of dead code only applies if we are using single driver mode
        controller.leftTrigger()
            .whileTrue(pieceControl.intakeNoteDriver(swerve::getPose, swerve::getRobotRelativeVelocity))
            .negate().and(() -> OIConstants.SINGLE_DRIVER_MODE || !operator.getLeftBumper())
            .onTrue(pieceControl.stopIntakeAndIndexer());
        
        controller.rightBumper()
            .onTrue(pieceControl.ejectNote())
            .onFalse(pieceControl.stopEjecting());

        // POV left and right are uncommonly used but needed incase of emergency
        controller.povLeft()
            .onTrue(pieceControl.stopAllMotors().andThen(pivot.setAngleCommand(60)));
        
        controller.povRight()
            .onTrue(Commands.runOnce(() -> pdh.setSwitchableChannel(false)));
    }

    private void configureSoloDiverBindings(PatriBoxController controller) {
        
        configureCommonDriverBindings(controller);

        controller.a()
            .toggleOnTrue(shooterCmds.preparePassCommand(swerve::getPose).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        controller.x()
            .whileTrue(pieceControl.sourceShooterIntake());

        controller.b()
            .onTrue(pieceControl.noteToTrap().andThen(elevator.toTopCommand()).andThen(pieceControl.prepPiece()));

        // If this is nice to work with, then we keep it. If not... bye bye!
        new Trigger(() -> elevator.getDesiredPosition() == ElevatorConstants.TRAP_PLACE_POS)
            .onTrue(swerve.resetHDCCommand())
            .whileTrue(alignmentCmds.ampRotationalAlignmentCommand(driver::getLeftX, driver::getLeftY));
        
        // Subwoofer preset incase something goes south
        controller.leftStick()
            .toggleOnTrue(shooterCmds.prepareSubwooferCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        
        // Quick uppies for double amping
        controller.leftBumper()
            .onTrue(elevator.toNoteFixCommand().alongWith(pieceControl.intakeUntilNote()))
            .onFalse(pieceControl.stopIntakeAndIndexer().andThen(elevator.toTopCommand().andThen(pieceControl.prepPiece())));

    }

    private void configureDriverBindings(PatriBoxController controller) {

        configureCommonDriverBindings(controller);

        controller.a()
            .onTrue(swerve.resetHDCCommand())
            .whileTrue(
                Commands.sequence(
                    Commands.either(
                        alignmentCmds.trapAlignmentCommand(controller::getLeftX, controller::getLeftY),
                        alignmentCmds.ampAlignmentCommand(controller::getLeftX),
                        climb::getHooksUp))
                    .alongWith(
                        limelight3.setLEDState(() -> true)))
            .onFalse(
                limelight3.setLEDState(() -> false));

        controller.x()
            .toggleOnTrue(
                alignmentCmds.preparePresetPose(driver::getLeftX, driver::getLeftY, true));

        controller.b()
            .toggleOnTrue(
                alignmentCmds.preparePresetPose(driver::getLeftX, driver::getLeftY, false));

        controller.leftBumper()
            .toggleOnTrue(shooterCmds.preparePassCommand(swerve::getPose)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    private void configureOperatorBindings(PatriBoxController controller) {
        controller.povUp()
            .onTrue(pieceControl.elevatorToPlacement(false));

        controller.povLeft()
            .onTrue(pieceControl.elevatorToPlacement(true));

        controller.povRight()
            .onTrue(ampper.toggleSpeed());

        controller.povDown()
            .onTrue(pieceControl.elevatorToBottom());

        controller.leftBumper()
            .whileTrue(pieceControl.intakeUntilNote())
            .negate().and(driver.leftTrigger().negate())
            .onTrue(pieceControl.stopIntakeAndIndexer());

        controller.rightBumper()
            .onTrue(pieceControl.ejectNote())
            .onFalse(pieceControl.stopEjecting());

        controller.rightTrigger()
            .onTrue(pieceControl
                .noteToTarget(swerve::getPose, swerve::getRobotRelativeVelocity, swerve::atHDCAngle,
                    () -> controller.getLeftBumper())
                .alongWith(driver.setRumble(() -> 0.5, 0.3))
                .andThen(Commands.runOnce(() -> RobotContainer.hasPiece = false)));

        controller.x()
            .onTrue(pieceControl.setShooterModeCommand(true));

        controller.b()
            .onTrue(pieceControl.setShooterModeCommand(false));

        controller.a()
            .onTrue(pieceControl.panicEjectNote())
            .onFalse(pieceControl.stopPanicEject());

        controller.start().or(controller.back()).or(controller.y())
            .whileTrue(pieceControl.sourceShooterIntake())
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.rightStick().and(controller.leftStick())
            .onTrue(elevator.resetEncoder());

        controller.rightStick().toggleOnTrue(
            elevator.overrideCommand(() -> Units.inchesToMeters(operator.getRightY())));

    }

    private void configureCalibrationBindings(PatriBoxController controller) {
        controller.leftBumper(testButtonBindingLoop)
            .onTrue(pieceControl.stopAllMotors().andThen(pivot.setAngleCommand(60)));
        controller.rightBumper(testButtonBindingLoop).onTrue(calibrationControl.updateMotorsCommand());
        controller.rightTrigger(0.5, testButtonBindingLoop)
            .onTrue(pieceControl.shootWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity, () -> true));

        controller.leftY(0.3, testButtonBindingLoop)
            .whileTrue(calibrationControl.incrementSpeeds(() -> (int) (controller.getLeftY() * 5)));
        controller.rightY(0.3, testButtonBindingLoop)
            .whileTrue(calibrationControl.incrementAngle(() -> controller.getRightY()));

        controller.leftX(0.3, testButtonBindingLoop)
            .whileTrue(calibrationControl.incrementLeftSpeed(() -> (int) (controller.getLeftX() * 5)));
        controller.rightX(0.3, testButtonBindingLoop)
            .whileTrue(calibrationControl.incrementRightSpeed(() -> (int) (controller.getRightX() * 5)));

        controller.back(testButtonBindingLoop).onTrue(calibrationControl.incrementDistance(-1));
        controller.start(testButtonBindingLoop).onTrue(calibrationControl.incrementDistance(1));

        controller.a(testButtonBindingLoop).onTrue(calibrationControl.logTriplet());

        controller.x(testButtonBindingLoop).onTrue(calibrationControl.toggleLeftLock());
        controller.b(testButtonBindingLoop).onTrue(calibrationControl.toggleRightLock());
        controller.y(testButtonBindingLoop).onTrue(calibrationControl.togglePivotLock());

        controller.pov(0, 270, testButtonBindingLoop)
            .whileTrue(pieceControl.intakeNoteDriver(swerve::getPose, swerve::getRobotRelativeVelocity)
                .alongWith(Commands.run(swerve::setWheelsX)))
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.pov(0, 90, testButtonBindingLoop)
            .onTrue(pieceControl.ejectNote());

        controller.pov(0, 0, testButtonBindingLoop)
            .whileTrue(pieceControl.sourceShooterIntake())
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.pov(0, 180, testButtonBindingLoop)
            .onTrue(calibrationControl.copyCalcTriplet());
    }
    
    private void configureHDCBindings(PatriBoxController controller) {
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
        enableVision();
        return pathPlannerStorage.getSelectedAuto();
    }

    public void onDisabled() {
        swerve.stopDriving();
        pieceControl.stopAllMotors().schedule();
        pathPlannerStorage.updatePathViewerCommand().schedule();
        fixPathPlannerCommands();

        limelight3.disableLEDS();

        // TODO: Extract this into a command file
        Commands.run(this::updateNTGains)
            .until(() -> Robot.gameMode != GameMode.DISABLED)
            .ignoringDisable(true)
            .schedule();

        driver.setRumble(0);
    }

    public void onEnabled() {
        if (Robot.gameMode == GameMode.TELEOP && !DriverStation.isFMSAttached()) {
            new LPI(ledStrip, swerve::getPose, driver, swerve::setDesiredPose).schedule();
        } else if (Robot.gameMode == GameMode.TEST) {
            CommandScheduler.getInstance().setActiveButtonLoop(testButtonBindingLoop);
        }
        gameModeStart = Robot.currentTimestamp;
        pathPlannerStorage.updatePathViewerCommand().schedule();
        this.freshCode = false;
    }

    public void updateNTGains() {
        double P = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0P").getDouble(-1);
        double I = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1I").getDouble(-1);
        double D = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2D").getDouble(-1);

        double P2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0P").getDouble(-1);
        double I2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1I").getDouble(-1);
        double D2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2D").getDouble(-1);

        if (P == -1 || I == -1 || D == -1 || P2 == -1 || I2 == -1 || D2 == -1) {
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0P").setDouble(AutoConstants.XY_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1I").setDouble(AutoConstants.XY_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2D").setDouble(AutoConstants.XY_CORRECTION_D);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0P").setDouble(AutoConstants.ROTATION_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1I").setDouble(AutoConstants.ROTATION_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2D").setDouble(AutoConstants.ROTATION_CORRECTION_D);
            return;
        }
        
        if (!(MathUtil.isNear(AutoConstants.HPFC.translationConstants.kP, P, 0.01)
                && MathUtil.isNear(AutoConstants.HPFC.translationConstants.kI, I, 0.01)
                && MathUtil.isNear(AutoConstants.HPFC.translationConstants.kD, D, 0.01)
                && MathUtil.isNear(AutoConstants.HPFC.rotationConstants.kP, P2, 0.01)
                && MathUtil.isNear(AutoConstants.HPFC.rotationConstants.kI, I2, 0.01)
                && MathUtil.isNear(AutoConstants.HPFC.rotationConstants.kD, D2, 0.01))) 
        {
            AutoConstants.HPFC = new HolonomicPathFollowerConfig(
                    new PIDConstants(
                            P,
                            I,
                            D),
                    new PIDConstants(
                            P2,
                            I2,
                            D2),
                    DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                    Math.hypot(DriveConstants.WHEEL_BASE, DriveConstants.TRACK_WIDTH) / 2.0,
                    new ReplanningConfig());
            
            swerve.reconfigureAutoBuilder();
            fixPathPlannerCommands();
            System.out.println("Reconfigured HPFC");
        }
    }

    private void prepareNamedCommands() {
        // TODO: prepare to shoot while driving (w1 - c1)
        NamedCommands.registerCommand("Intake", pieceControl.intakeAuto());
        NamedCommands.registerCommand("ToIndexer", pieceControl.intakeUntilNote());
        NamedCommands.registerCommand("StopIntake", pieceControl.stopIntakeAndIndexer());
        NamedCommands.registerCommand("StopAll", pieceControl.stopAllMotors());
        NamedCommands.registerCommand("PrepareShooter", shooterCmds.prepareFireCommandAuto(swerve::getPose));
        NamedCommands.registerCommand("Shoot", pieceControl.noteToShoot(swerve::getPose, swerve::getRobotRelativeVelocity));
        NamedCommands.registerCommand("ShootInstantly", pieceControl.noteToShootUsingSensor(swerve::getPose, swerve::getRobotRelativeVelocity));
        NamedCommands.registerCommand("ShootInstantlyWhenReady", Commands.waitSeconds(.4).andThen(pieceControl.noteToShootUsingSensorWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity)));
        NamedCommands.registerCommand("ShootWhenReady", pieceControl.shootPreload());
        NamedCommands.registerCommand("RaiseElevator", elevator.toTopCommand());
        NamedCommands.registerCommand("LowerElevator", elevator.toBottomCommand());
        NamedCommands.registerCommand("PlaceAmp", pieceControl.placeWhenReady());
        NamedCommands.registerCommand("PrepareShooterL", shooterCmds.prepareFireCommand(() -> FieldConstants.L_POSE, Robot::isRedAlliance));
        NamedCommands.registerCommand("PrepareShooterM", shooterCmds.prepareFireCommand(() -> FieldConstants.M_POSE, Robot::isRedAlliance));
        NamedCommands.registerCommand("PrepareShooterR", shooterCmds.prepareFireCommand(() -> FieldConstants.R_POSE, Robot::isRedAlliance));
        NamedCommands.registerCommand("PrepareShooterW3", shooterCmds.prepareFireCommand(() -> FieldConstants.W3_POSE, Robot::isRedAlliance));
        NamedCommands.registerCommand("PrepareShooter", shooterCmds.prepareFireCommand(pathPlannerStorage::getNextShotTranslation, () -> false));
        NamedCommands.registerCommand("PrepareSWD", shooterCmds.prepareSWDCommandAuto(swerve::getPose, swerve::getRobotRelativeVelocity));
        NamedCommands.registerCommand("DisableLimelight", disableVision());
        NamedCommands.registerCommand("EnableLimelight", enableVision());
        registerPathToPathCommands();
    }

    private Command disableVision() {
        return Commands.runOnce(() -> enableVision = false).ignoringDisable(true);
    }

    private Command enableVision() {
        return Commands.runOnce(() -> enableVision = true).ignoringDisable(true);
    }

    private void registerPathToPathCommands() {
        for (int i = 1; i <= FieldConstants.CENTER_NOTE_COUNT; i++) {
            for (int j = 1; j <= FieldConstants.CENTER_NOTE_COUNT; j++) {
                if (i == j) {
                    continue;
                }
                NamedCommands.registerCommand("C" + i + "toC" + j, pathPlannerStorage.generateCenterLogic(i, j, swerve, limelight2));
            }
        }
    }

    private void configureLoggingPaths() {
        Monologue.logObj(shooterCalc, "Robot/Math/shooterCalc");
        Monologue.logObj(calibrationControl, "Robot/Math/calibrationControl");
        Monologue.logObj(HDCTuner, "Robot/Math/HDCTuner");
        Monologue.logObj(pieceControl, "Robot/Math/PieceControl");

        Monologue.logObj(swerve, "Robot/Swerve");

        Monologue.logObj(intake, "Robot/Subsystems/intake");
        Monologue.logObj(climb, "Robot/Subsystems/climb");
        if (CameraConstants.FIELD_CALIBRATION_MODE) {
            Monologue.logObj(limelightMapper, "Robot/Limelights/limelightMapper");
        } else {
            Monologue.logObj(limelight2, "Robot/Limelights/limelight2");
            Monologue.logObj(limelight3, "Robot/Limelights/limelight3");
        }
        Monologue.logObj(colorSensor, "Robot/ColorSensors/colorSensor");
        Monologue.logObj(shooter, "Robot/Subsystems/shooter");
        Monologue.logObj(elevator, "Robot/Subsystems/elevator");
        Monologue.logObj(pivot, "Robot/Subsystems/pivot");
        Monologue.logObj(ampper, "Robot/Subsystems/ampper");
        Monologue.logObj(pieceControl, "Robot/Managers/pieceControl");
        
        Monologue.logObj(pathPlannerStorage, "Robot");
    }

    /**
     * This is explained in detail in discussion #180 of the 2024 repository of 4738
     * https://github.com/Patribots4738/Crescendo2024/discussions/180
     */
    private void fixPathPlannerCommands() {
        registerPathToPathCommands();
        pathPlannerStorage.configureAutoChooser();
    }

    private void initializeComponents() {

        Pose3d initialShooterPose = new Pose3d(
                NTConstants.PIVOT_OFFSET_METERS.getX(),
                0,
                NTConstants.PIVOT_OFFSET_METERS.getZ(),
            new Rotation3d(0, -Units.degreesToRadians(ShooterConstants.PIVOT_LOWER_LIMIT_DEGREES), 0)
        );

        components3d[0] = initialShooterPose;
        desiredComponents3d[0] = initialShooterPose;

        for (int i = 1; i < components3d.length; i++) {
            components3d[i] = new Pose3d();
            desiredComponents3d[i] = new Pose3d();
        }

        notePose3ds[0] = new Pose3d();
        highNotePose3ds[0] = new Pose3d(0,0,-0.1, new Rotation3d());
        for (int i = 1; i < notePose3ds.length; i++) {
            notePose3ds[i] = new Pose3d(FieldConstants.NOTE_TRANSLATIONS[i-1], new Rotation3d());
            highNotePose3ds[i] = new Pose3d(FieldConstants.HIGH_NOTE_TRANSLATIONS[i-1], new Rotation3d());
        }
    }

    public void turnOffLamp() {
        pdh.setSwitchableChannel(false);
        limelight3.disableLEDS();
    }
}
 
