package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.ampper.Ampper;
import frc.robot.subsystems.ampper.Elevator;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.colorsensor.PicoColorSensor;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.CameraConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.calc.LimelightMapping;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.calc.ShooterCalc;
import frc.robot.util.custom.PatriBoxController;
import frc.robot.util.custom.ActiveConditionalCommand;
import frc.robot.util.rev.Neo;

public class RobotContainer {

    private PowerDistribution pdh;

    //* We add a event loop here so we can use different bindings in a different mode
    //* of the code (there are too many binding to fit on the controller)
    private EventLoop testButtonBindingLoop = new EventLoop();
    
    private final PatriBoxController driver;
    private final PatriBoxController operator;

    private Swerve swerve;
    private final Intake intake;
    private Limelight limelight3g;
    private Limelight limelight3;
    private LimelightMapping limelightMapper;
    private final Climb climb;
    private Pivot pivot;
    private Shooter shooter;
    private Elevator elevator;
    
    private CalibrationControl calibrationControl;
    private PathPlannerStorage pathPlannerStorage;
    private ShooterCalc shooterCalc;
    public static HDCTuner HDCTuner;

    private Indexer indexer;
    private Ampper ampper;
    private ShooterCmds shooterCmds;
    private PicoColorSensor piPico;

    private PieceControl pieceControl;
    private AlignmentCmds alignmentCmds;
    private boolean fieldRelativeToggle = true;
    private final BooleanSupplier robotRelativeSupplier;
    
    @AutoLogOutput (key = "Draggables/Components3d")
    public static Pose3d[] components3d = new Pose3d[5];
    @AutoLogOutput (key = "Draggables/DesiredComponents3d")
    public static Pose3d[] desiredComponents3d = new Pose3d[5];
    @AutoLogOutput (key = "Draggables/NotePose3ds")
    public static Pose3d[] notePose3ds = new Pose3d[12];
    @AutoLogOutput (key = "Draggables/HighNotePose3ds")
    public static Pose3d[] highNotePose3ds = new Pose3d[12];
    @AutoLogOutput (key = "Draggables/FreshCode")
    private boolean freshCode = true;
    @AutoLogOutput (key = "Draggables/RobotPose2d")
    public static Pose2d robotPose2d = new Pose2d();
    @AutoLogOutput (key = "Draggables/VisionErrorPose")
    public static Transform2d visionErrorPose = new Transform2d();
    @AutoLogOutput (key = "Draggables/DistanceToSpeakerMeters")
    public static double distanceToSpeakerMeters = 0;
    @AutoLogOutput (key = "Draggables/DistanceToSpeakerFeet")
    public static double distanceToSpeakerFeet = 0;
    @AutoLogOutput (key = "Draggables/RobotPose3d")
    public static Pose3d robotPose3d = new Pose3d();
    @AutoLogOutput (key = "Draggables/SwerveMeasuredStates")
    public static SwerveModuleState[] swerveMeasuredStates;
    @AutoLogOutput (key = "Draggables/SwerveDesiredStates")
    public static SwerveModuleState[] swerveDesiredStates;
    @AutoLogOutput (key = "Draggables/GameModeStart")
    public static double gameModeStart = 0;
    @AutoLogOutput (key = "Draggables/HasPiece")
    public static boolean hasPiece = true;
    @AutoLogOutput (key = "Draggables/EnableVision")
    public static boolean enableVision = true;
    @AutoLogOutput (key = "Draggables/Timer")
    public static double displayTime = 0.0;
    @AutoLogOutput (key = "Draggables/AutoStatingPose")
    public static Pose2d autoStartingPose = new Pose2d();

    public static Field2d field2d = new Field2d();
    
    public RobotContainer() {

        System.out.println("Constructing Robot Container...");
        
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);
        
        pdh = new PowerDistribution(30, ModuleType.kRev);
        pdh.setSwitchableChannel(false);

        piPico = new PicoColorSensor();

        intake = new Intake();
        climb = new Climb();
        swerve = new Swerve();
        if (CameraConstants.FIELD_CALIBRATION_MODE) {
            limelight3g = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "not-limelight-threeg", 0);
            limelight3 = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "not-limelight-three", 1);
            limelightMapper = new LimelightMapping(swerve.getPoseEstimator(), swerve::getPose, "limelight-threeg");
            
            // limelightMapper.ManuallyAddPose(12, new Pose3d(11.173, 4.096, 1.443, new Rotation3d(new Quaternion(0.003, 0.032, -0.025, -0.999))));
            // limelightMapper.ManuallyAddPose(2, new Pose3d(16.787, 5.036, 1.562, new Rotation3d(new Quaternion(.00099, -0.019, -0.017, -1.000))));
            // limelightMapper.ManuallyAddPose(3, new Pose3d(16.776, 5.599, 1.562, new Rotation3d(new Quaternion(0.009, 0.016, -0.003, -1.000))));
            // limelightMapper.printJSON();
        } else {
            limelight3g = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "limelight-threeg", 0);
            limelight3 = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "limelight-three", 1);
            limelight3.disableLEDS();
            limelight3g.disableLEDS();
        }

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
            piPico);

        calibrationControl = new CalibrationControl(shooterCmds);

        driver.back().toggleOnTrue(
            Commands.runOnce(() -> fieldRelativeToggle = !fieldRelativeToggle)
        );
        robotRelativeSupplier = () -> fieldRelativeToggle;

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX()/1.6,
            robotRelativeSupplier,
            () -> (robotRelativeSupplier.getAsBoolean() && Robot.isRedAlliance())
        ));

        shooter.setDefaultCommand(
            pieceControl.getAutomaticShooterSpeeds(
                swerve::getPose,
                () -> driver.getXButton() 
                || (OIConstants.OPERATOR_PRESENT 
                    && operator.getLeftBumper()),
                () -> OIConstants.OPERATOR_PRESENT 
                && (driver.getLeftBumper() || operator.getYButton())
            )
        );

        if (FieldConstants.IS_SIMULATION) {
            pathPlannerStorage = new PathPlannerStorage(operator::getYButton, swerve, limelight3);
        } else {
            pathPlannerStorage = new PathPlannerStorage(piPico::hasNoteShooter, piPico::hasNoteElevator, swerve, limelight3);
        }
        
        initializeComponents();
        prepareNamedCommands();

        SmartDashboard.putData(field2d);

        pathPlannerStorage.configureAutoChooser();
        pathPlannerStorage.getAutoChooser().addOption("WheelRadiusCharacterization",
            disableVision()
            .andThen(
                swerve.setWheelsOCommand()
                .andThen(new WheelRadiusCharacterization(swerve)))
            .andThen(enableVision()));
        
        configureButtonBindings();

        pdh.setSwitchableChannel(false);

    }
    
    private void configureButtonBindings() {

        if (CameraConstants.FIELD_CALIBRATION_MODE) {
            configureFieldCalibrationBindings(driver);
            return;
        }

        switch (OIConstants.DRIVER_MODE) {
            case SINGLE:
                configureSoloDriverBindings(driver);
                break;
            case DOUBLE:
                configureDriverBindings(driver);
                configureOperatorBindings(operator);
                break;
            case DEV:
                configureDevDriverBindings(driver);
                break;
        }
        

        configureTimedEvents();
        configureTestBindings();
    }

    private void configureTestBindings() {
        // Warning: these buttons are not on the default loop!
        // See https://docs.wpilib.org/en/stable/docs/software/convenience-features/event-based.html
        // for more information 
        // configureHDCBindings(driver);
        configureCalibrationBindings(operator, testButtonBindingLoop);
        // configureCalibrationBindings(operator, CommandScheduler.getInstance().getDefaultButtonLoop());
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
            && shooter.getAverageSpeed() > 1700
            && shooter.getAverageTargetSpeed() != 2500
            && swerve.getPose().getX() > FieldConstants.CENTERLINE_X ^ Robot.isBlueAlliance()
            && limelight3g.getPose2d().getTranslation().getDistance(swerve.getPose().getTranslation()) < Units.inchesToMeters(4))
        .onTrue(Commands.runOnce(() -> 
            pdh.setSwitchableChannel(true)).alongWith(limelight3g.blinkLeds(() -> 1), driver.setRumble(() -> 1)))
        .onFalse(Commands.runOnce(() -> 
            pdh.setSwitchableChannel(false)).alongWith(driver.setRumble(() -> 0)));
        
        // When our alliance changes, reflect that in the path previewer
        new Trigger(Robot::isRedAlliance)
            .onTrue(pathPlannerStorage.updatePathViewerCommand())
            .onFalse(pathPlannerStorage.updatePathViewerCommand());
        
        new Trigger(() -> PoseCalculations.isAlignedToAmp(robotPose2d)).or(shooterCalc.readyToShootSupplier())
            .onTrue(driver.setRumble(() -> 0.5))
            .onFalse(driver.setRumble(() -> 0));
        
        // Make the controllers pulse just like the LEDs do
        // The reason we are checking bumpers
        // is so this doesn't happen on pieceControl::moveNote
        new Trigger(
            () -> piPico.hasNoteShooter() 
                && (
                    // All of these buttons command intake
                    // Whether that be source or not
                    driver.getLeftTrigger() 
                    || driver.getXButton() 
                    || (OIConstants.OPERATOR_PRESENT 
                        &&(operator.getLeftBumper() 
                        || operator.getStartButton() 
                        || operator.getBackButton()))
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
                .alongWith(limelight3g.blinkLeds(() -> 1))
                .alongWith(limelight3.blinkLeds(() -> 1))
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
        
        // Controller rumble if climb hooks are up and we are inside of stage area
        // AKA driver is good to climb
        new Trigger(climb::getHooksUp).and(() -> PoseCalculations.inStageTriangle(robotPose2d))
            .onTrue(driver.setRumble(() -> 0.5))
            .onFalse(driver.setRumble(() -> 0));

        new Trigger(climb::getToggleMode)
            .onTrue(swerve.resetHDCCommand())
            .onFalse(swerve.resetDesiredHDCPoseCommand())
            .whileTrue(alignmentCmds.chainRotationalAlignment(driver::getLeftX, driver::getLeftY, robotRelativeSupplier));  
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
        controller.start()
            .onTrue(Commands.runOnce(() -> 
                swerve.resetOdometry(FieldConstants.GET_SUBWOOFER_POSITION()), swerve));

        // Speaker / Source / Chain rotational alignment
        controller.rightStick()
            .onTrue(swerve.resetHDCCommand())
            .toggleOnTrue(
                Commands.sequence(
                    pieceControl.elevatorToBottomSafe()
                        .unless(() -> elevator.getDesiredPosition() == ElevatorConstants.BOTTOM_POS),
                    limelight3g.setLEDState(() -> true),
                    new ActiveConditionalCommand(
                        // This runs SWD on heading control 
                        // and shooting-while-still on shooter
                        alignmentCmds.wingRotationalAlignment(controller::getLeftX, controller::getLeftY, robotRelativeSupplier),
                        alignmentCmds.preparePassCommand(controller::getLeftX, controller::getLeftY, robotRelativeSupplier),
                        () -> PoseCalculations.inSpeakerShotZone(robotPose2d.getTranslation()) || climb.getHooksUp())
                    ).finallyDo(
                        () -> 
                            Commands.parallel(
                                swerve.resetDesiredHDCPoseCommand(),
                                limelight3g.setLEDState(() -> false),
                                Commands.parallel(
                                    shooterCmds.raisePivot(),
                                    shooterCmds.stopShooter())
                                    .withInterruptBehavior(InterruptionBehavior.kCancelSelf))
                                .schedule()));
        
        // Note to target will either place amp or shoot,
        // depending on if the elevator is up or not
        controller.rightTrigger()
            .onTrue(
                pieceControl.noteToTarget(
                    swerve::getPose, 
                    swerve::getRobotRelativeVelocity, 
                    swerve::atHDCAngle, 
                    operator::getLeftBumper)
                    .alongWith(driver.setRumble(() -> 0.5, 0.3))
            .andThen(Commands.runOnce(() -> RobotContainer.hasPiece = false)));
      
        controller.rightBumper()
            .onTrue(pieceControl.ejectNote())
            .negate().and(() -> !OIConstants.OPERATOR_PRESENT  || !operator.getRightBumper())
            .onTrue(pieceControl.stopEjecting());
        
        // Climbing controls
        controller.povUp()
            .onTrue(climb.povUpCommand());

        controller.povDown()
            .onTrue(
                climb.toBottomCommand()
                    .alongWith(
                        Commands.waitUntil(elevator::atDesiredPosition)
                            .andThen(shooterCmds.stowPivot())));

        // POV left is uncommonly used but needed incase of emergency
        controller.povLeft()
            .onTrue(
                Commands.sequence(
                    pieceControl.stopAllMotors(),
                    Commands.waitUntil(elevator::atDesiredPosition),
                    shooterCmds.raisePivot()
                ));

    }

    private void configureSoloDriverBindings(PatriBoxController controller) {
        
        configureCommonDriverBindings(controller);

        controller.a()
            .whileTrue(pieceControl.blepNote())
            .onFalse(pieceControl.stopIntakeAndIndexer().alongWith(shooterCmds.raisePivot()));

        controller.leftBumper()
            .whileTrue(pieceControl.sourceShooterIntake())
            .onFalse(pieceControl.stopIntakeAndIndexer().alongWith(shooterCmds.raisePivot()));

        controller.b()
            .onTrue(pieceControl.noteToTrap3()
                .andThen(
                    pieceControl.elevatorUpWhileIntaking(), 
                    pieceControl.prepPiece()));

        // If this is nice to work with, then we keep it. If not... bye bye!
        new Trigger(() -> elevator.getDesiredPosition() == ElevatorConstants.TRAP_PLACE_POS 
                    && swerve.getPose().getY() > FieldConstants.FIELD_HEIGHT_METERS/2.0)
            .onTrue(swerve.resetHDCCommand())
            .onFalse(swerve.resetDesiredHDCPoseCommand())
            .whileTrue(alignmentCmds.ampRotationalAlignmentCommand(driver::getLeftX, driver::getLeftY));
        
        // Subwoofer preset incase something goes south
        controller.leftStick()
            .toggleOnTrue(shooterCmds.prepareSubwooferCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        
        // Quick uppies for double amping
        controller.y()
            .onTrue(shooterCmds.raisePivot().alongWith(elevator.toNoteFixCommand().alongWith(pieceControl.intakeForDoubleAmp())))
            .onFalse(ampper.outtakeSlow(.3).andThen(pieceControl.stopIntakeAndIndexer(),pieceControl.doubleAmpElevatorEnd()));

        // controller.leftBumper()
        //     .onTrue(elevator.toTopIshButNotFullCommand())
        //     .onFalse(elevator.toBottomCommand());

        controller.leftBumper()
            .whileTrue(pieceControl.intakeAuto())
            .onFalse(pieceControl.stopIntakeAndIndexer());

    }

    private void configureDriverBindings(PatriBoxController controller) {

        configureCommonDriverBindings(controller);

        controller.leftStick()
            .toggleOnTrue(
                Commands.sequence(
                    pieceControl.elevatorToBottomSafe(),
                    shooterCmds.prepareSubwooferCommand()
                ).finallyDo(
                    () -> 
                    Commands.parallel(
                        shooterCmds.raisePivot(),
                        shooterCmds.stopShooter())
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                        .schedule()));


        controller.a()
            .onTrue(swerve.resetHDCCommand())
            .whileTrue(
                alignmentCmds.ampRotationalAlignmentCommand(controller::getLeftX, controller::getLeftY)
                    .alongWith(limelight3g.setLEDState(() -> true)))
            .onFalse(
                Commands.parallel(
                    limelight3g.setLEDState(() -> false),
                    swerve.resetHDCCommand(),
                    swerve.resetDesiredHDCPoseCommand()));
                
        controller.x()
            .whileTrue(pieceControl.intakeNoteDriver(swerve::getPose, swerve::getRobotRelativeVelocity))
            .negate().and(() -> !OIConstants.OPERATOR_PRESENT  || !operator.getLeftBumper())
            .onTrue(pieceControl.stopIntakeAndIndexer());

        controller.y()
            .onTrue(
                Commands.either( 
                    climb.toTopCommand().alongWith(climb.setToggleMode(true)),
                    climb.toBottomCommand().alongWith(shooterCmds.stowPivot()),
                    () -> !climb.getHooksUp()));

        controller.leftTrigger()
            .onTrue(pieceControl.blepNote());
        
    }

    private void configureOperatorBindings(PatriBoxController controller) {
        controller.povUp()
            .onTrue(pieceControl.elevatorToPlacement(false));

        controller.povLeft()
            .onTrue(pieceControl.elevatorToPlacement(true));

        controller.povRight()
            .onTrue(ampper.toggleSpeed());

        controller.povDown()
            .onTrue(pieceControl.elevatorToBottomSafe());

        controller.leftBumper()
            .whileTrue(pieceControl.intakeUntilNote())
            .negate().and(driver.x().negate())
            .onTrue(pieceControl.stopIntakeAndIndexer());

        controller.rightBumper()
            .onTrue(pieceControl.ejectNote())
            .negate().and(driver.rightBumper().negate())
            .onTrue(pieceControl.stopEjecting());

        controller.rightTrigger()
            .onTrue(pieceControl
                .noteToTarget(
                    swerve::getPose, 
                    swerve::getRobotRelativeVelocity, 
                    () -> swerve.atHDCAngle(),
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

        controller.start().or(controller.back())
            .whileTrue(pieceControl.sourceShooterIntake())
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.rightStick().and(controller.leftStick())
            .onTrue(elevator.resetEncoder());

        controller.rightStick().toggleOnTrue(
            elevator.overrideCommand(() -> Units.inchesToMeters(operator.getRightY())));

    }

    private void configureDevDriverBindings(PatriBoxController controller) {

        configureCommonDriverBindings(controller);

        controller.x()
            .onTrue(
                swerve.resetOdometryCommand(FieldConstants::GET_SAMPLE_CENTER_PASS_POSITION));

        controller.b()  
            .onTrue(
                swerve.resetOdometryCommand(FieldConstants::GET_SAMPLE_SOURCE_PASS_POSITION));

        controller.leftTrigger()
            .whileTrue(pieceControl.sourceShooterIntake())
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.y()
            .whileTrue(pieceControl.intakeNoteDriver(swerve::getPose, swerve::getRobotRelativeVelocity))
            .negate().and(() -> !OIConstants.OPERATOR_PRESENT  || !operator.getLeftBumper())
            .onTrue(pieceControl.stopIntakeAndIndexer());
            

    }

    private void configureCalibrationBindings(PatriBoxController controller, EventLoop eventLoop) {
        controller.leftBumper(eventLoop)
            .onTrue(pieceControl.stopAllMotors().andThen(shooterCmds.raisePivot()));
        controller.rightBumper(eventLoop).onTrue(calibrationControl.updateMotorsCommand());
        controller.rightTrigger(0.5, eventLoop)
            .onTrue(pieceControl.shootWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity, () -> true));

        controller.leftY(0.3, eventLoop)
            .whileTrue(calibrationControl.incrementSpeeds(() -> (int) (controller.getLeftY() * 5)));
        controller.rightY(0.3, eventLoop)
            .whileTrue(calibrationControl.incrementAngle(() -> controller.getRightY()));

        controller.leftX(0.3, eventLoop)
            .whileTrue(calibrationControl.incrementLeftSpeed(() -> (int) (controller.getLeftX() * 5)));
        controller.rightX(0.3, eventLoop)
            .whileTrue(calibrationControl.incrementRightSpeed(() -> (int) (controller.getRightX() * 5)));

        controller.back(eventLoop).onTrue(calibrationControl.incrementDistance(-0.5));
        controller.start(eventLoop).onTrue(calibrationControl.incrementDistance(0.5));

        controller.a(eventLoop).onTrue(calibrationControl.logTriplet());

        // controller.x(eventLoop).onTrue(calibrationControl.toggleLeftLock());
        // controller.b(eventLoop).onTrue(calibrationControl.toggleRightLock());
        // controller.y(eventLoop).onTrue(calibrationControl.togglePivotLock());

        controller.pov(0, 270, eventLoop)
            .whileTrue(pieceControl.intakeNoteDriver(swerve::getPose, swerve::getRobotRelativeVelocity)
                .alongWith(Commands.run(swerve::setWheelsX)))
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.pov(0, 90, eventLoop)
            .onTrue(pieceControl.ejectNote());

        controller.pov(0, 0, eventLoop)
            .whileTrue(pieceControl.sourceShooterIntake())
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.pov(0, 180, eventLoop)
            .onTrue(calibrationControl.copyCalcTriplet(swerve::getPose));
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

        limelight3g.disableLEDS();

        // TODO: Extract this into a command file
        Commands.run(this::updateNTGains)
            .until(() -> Robot.gameMode != GameMode.DISABLED)
            .ignoringDisable(true)
            .schedule();

        driver.setRumble(0);
    }

    public void onEnabled() {
        if (Robot.gameMode == GameMode.TEST) {
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
            
            // swerve.reconfigureAutoBuilder();
            // fixPathPlannerCommands();
            // System.out.println("Reconfigured HPFC");
        }
    }

    private void prepareNamedCommands() {
        NamedCommands.registerCommand("Intake", pieceControl.intakeAuto());
        // If you can fix this formatting I salute you
        NamedCommands.registerCommand("ToIndexer",
            Commands.either(
                pieceControl.intakeUntilNote(),
                Commands.waitUntil(operator::getYButton),
                () -> !FieldConstants.IS_SIMULATION
            )
        );
        NamedCommands.registerCommand("StopIntake", pieceControl.stopIntakeAndIndexer());
        NamedCommands.registerCommand("StopAll", pieceControl.stopAllMotors());
        NamedCommands.registerCommand("PrepareShooter", shooterCmds.prepareFireCommandAuto(swerve::getPose));
        NamedCommands.registerCommand("Shoot", pieceControl.noteToShoot(swerve::getPose, swerve::getRobotRelativeVelocity));
        NamedCommands.registerCommand("ShootInstantly", 
            Commands.either(
                pieceControl.noteToShootUsingSensor(swerve::getPose, swerve::getRobotRelativeVelocity),
                Commands.waitUntil(() -> !operator.getYButton()), 
                () -> !FieldConstants.IS_SIMULATION
            )
        );
        // This one too, what a necessary nightmare :(
        NamedCommands.registerCommand("ShootInstantlyWhenReady",
            Commands.waitSeconds(.4)
                .andThen(
                    Commands.either(
                        pieceControl.noteToShootUsingSensorWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity),
                        Commands.waitUntil(() -> !operator.getYButton()), 
                        () -> !FieldConstants.IS_SIMULATION
                    )
                )
            );
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
        NamedCommands.registerCommand("FullPowerPreload", 
            shooter.fullPower(1678) // Say that again?
                .alongWith(Commands.waitUntil(pivot::getAtDesiredAngle))
                .andThen(pieceControl.intakeAuto()
                    .alongWith(shooterCmds.getNoteTrajectoryCommand(swerve::getPose, swerve::getRobotRelativeVelocity)))
                .deadlineWith(shooterCmds.preparePivotCommandAuto(swerve::getPose, swerve::getRobotRelativeVelocity)));

        NamedCommands.registerCommand("FullPowerPreload2",
            shooter.fullPower(2200)
                .alongWith(Commands.waitUntil(pivot::getAtDesiredAngle))
                .andThen(
                    Commands.either(
                        pieceControl.noteToShootUsingSensor(swerve::getPose, swerve::getRobotRelativeVelocity),
                        Commands.waitUntil(() -> !operator.getYButton()), 
                        () -> !FieldConstants.IS_SIMULATION
                    )
                )
                .deadlineWith(shooterCmds.preparePivotCommandAuto(swerve::getPose, swerve::getRobotRelativeVelocity)));

        NamedCommands.registerCommand("FullPowerPreload3",
            shooter.fullPower(1678) // Say that again?
                .alongWith(Commands.waitUntil(pivot::getAtDesiredAngle))
                .andThen(
                    Commands.either(
                        pieceControl.noteToShootUsingSensor(swerve::getPose, swerve::getRobotRelativeVelocity),
                        Commands.waitUntil(() -> !operator.getYButton()), 
                        () -> !FieldConstants.IS_SIMULATION
                    )
                )
                .deadlineWith(shooterCmds.preparePivotCommandAuto(swerve::getPose, swerve::getRobotRelativeVelocity)));
        NamedCommands.registerCommand("BoostShooterR",
            shooter.fullPower(2500)
            .raceWith(shooterCmds.preparePivotCommandAuto(swerve::getPose, swerve::getRobotRelativeVelocity)));
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
                NamedCommands.registerCommand("C" + i + "toC" + j, pathPlannerStorage.generateCenterLogicOBJ(i, j, swerve, limelight3));
                NamedCommands.registerCommand("C" + i + "toC" + j + "nonOBJ", pathPlannerStorage.generateCenterLogicNonOBJ(i, j, swerve));
            }
        }
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
        limelight3g.disableLEDS();
    }
}
 
