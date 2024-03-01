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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.GameMode;
import frc.robot.commands.drive.AlignmentCmds;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.DriveHDC;
import frc.robot.commands.logging.NT;
import frc.robot.commands.logging.NTPIDTuner;
import frc.robot.commands.managers.CalibrationControl;
import frc.robot.commands.managers.HDCTuner;
import frc.robot.commands.managers.PieceControl;
import frc.robot.commands.managers.ShooterCmds;
import frc.robot.leds.Commands.LPI;
import frc.robot.leds.Strips.LedStrip;
import frc.robot.subsystems.*;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.calc.ShooterCalc;
import frc.robot.util.custom.PatriBoxController;
import frc.robot.util.custom.ActiveConditionalCommand;
import frc.robot.util.rev.Neo;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

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
    private Indexer triggerWheel;
    private Trapper trapper;
    private ShooterCmds shooterCmds;

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
    public static Pose3d robotPose3d = new Pose3d();
    @Log
    public static SwerveModuleState[] swerveMeasuredStates;
    @Log
    public static SwerveModuleState[] swerveDesiredStates;
    
    public RobotContainer() {
        
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);
        
        intake = new Intake();
        climb = new Climb();
        swerve = new Swerve();
        limelight3 = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "limelight");
        limelight2 = new Limelight(swerve.getPoseEstimator(), swerve::getPose, "limelight2");
        ledStrip = new LedStrip(swerve::getPose);
        triggerWheel = new Indexer();

        shooter = new Shooter();
        elevator = new Elevator();
        trapper = new Trapper();
        
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
            triggerWheel,
            elevator,
            trapper,
            shooterCmds);

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

        pathPlannerStorage = new PathPlannerStorage(driver.y().negate());
        initializeComponents();
        prepareNamedCommands();
        // choreoPathStorage = new ChoreoStorage(driver.y());
        // setupChoreoChooser();
        pathPlannerStorage.configureAutoChooser();
        
        configureButtonBindings();
        configureLoggingPaths();
    }

    private void configureButtonBindings() {
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
        // configureHDCBindings(driver);
        configureCalibrationBindings(operator);

    }
    
    private void configureDriverBindings(PatriBoxController controller) {
        
        // Upon hitting start or back,
        // reset the orientation of the robot
        // to be facing AWAY FROM the driver station
        controller.back().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    Robot.isRedAlliance() ? FieldConstants.FIELD_WIDTH_METERS - 1.31 : 1.31,
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
            .onTrue(shooterCmds.stowPivot())
            .toggleOnTrue(climb.povUpCommand(swerve::getPose));
        
        controller.povDown()
            .onTrue(shooterCmds.stowPivot()
            .alongWith(climb.toBottomCommand())
        );
        
        controller.a().whileTrue(
            Commands.sequence(
                swerve.resetHDCCommand(),
                new ActiveConditionalCommand(
                    alignmentCmds.trapAlignmentCommand(controller::getLeftX, controller::getLeftY),
                    alignmentCmds.ampAlignmentCommand(controller::getLeftX), 
                    climb::getHooksUp)));
                    
        controller.x()
            .toggleOnTrue(alignmentCmds.preparePresetPose(driver::getLeftX, driver::getLeftY, true));

        controller.b()
            .toggleOnTrue(alignmentCmds.preparePresetPose(driver::getLeftX, driver::getLeftY, false));
        
        controller.rightTrigger()
            .onTrue(pieceControl.noteToTarget(swerve::getPose, swerve::getRobotRelativeVelocity));

        controller.rightStick()
            .toggleOnTrue(
                Commands.sequence(
                    swerve.resetHDCCommand(),
                    new ActiveConditionalCommand(
                        alignmentCmds.sourceRotationalAlignment(controller::getLeftX, controller::getLeftY, robotRelativeSupplier),
                        alignmentCmds.wingRotationalAlignment(controller::getLeftX, controller::getLeftY, robotRelativeSupplier),
                        alignmentCmds.alignmentCalc::onOppositeSide)));

        controller.povLeft()
            .onTrue(pieceControl.stopAllMotors());
            
        controller.leftBumper()
            .onTrue(pieceControl.intakeToTrap())
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.rightBumper()
            .onTrue(pieceControl.ejectNote())
            .onFalse(pieceControl.stopEjecting());
    }

    private void configureOperatorBindings(PatriBoxController controller) {
        controller.povUp()
            .onTrue(pieceControl.elevatorToTop());

        controller.povLeft()
            .onTrue(pieceControl.elevatorToAmp());

        controller.povRight()
            .onTrue(trapper.toggleSpeed());
        
        controller.povDown()
            .onTrue(elevator.toBottomCommand());

        controller.leftBumper()
            .whileTrue(pieceControl.intakeToTrap())
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.rightBumper()
            .onTrue(pieceControl.ejectNote())
            .onFalse(pieceControl.stopEjecting());

        controller.rightTrigger()
            .onTrue(
                shooter.setSpeedCommand(3000)
                    .alongWith(pivot.setAngleCommand(30))
                .andThen(Commands.waitUntil(shooterCalc.readyToShootSupplier()))
                .andThen(pieceControl.noteToShoot(swerve::getPose, swerve::getRobotRelativeVelocity))
                .andThen(shooter.stopCommand()
                    .alongWith(pivot.setAngleCommand(0)))
            );

        controller.x()
            .onTrue(pieceControl.setShooterModeCommand(true));

        controller.b()
            .onTrue(pieceControl.setShooterModeCommand(false));
        controller.a()
            .onTrue(pieceControl.panicEjectNote())
            .onFalse(pieceControl.stopPanicEject());
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
            .whileTrue(pieceControl.intakeToTrap())
            .onFalse(pieceControl.stopIntakeAndIndexer());

        controller.pov(0, 90, testButtonBindingLoop)
            .onTrue(pieceControl.ejectNote());

        controller.pov(0, 0, testButtonBindingLoop)
            .onTrue(pieceControl.stopIntakeAndIndexer());

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
        return pathPlannerStorage.getSelectedAuto();
    }

    public void onDisabled() {
        swerve.stopDriving();
        pieceControl.stopAllMotors().schedule();
        pathPlannerStorage.updatePathViewerCommand().schedule();
        fixPathPlannerCommands();

        // TODO: Extract this into a command file
        Commands.run(this::updateNTGains)
            .until(() -> Robot.gameMode != GameMode.DISABLED)
            .ignoringDisable(true)
            .schedule();
    }

    public void onEnabled() {
        if (Robot.gameMode == GameMode.TELEOP) {
            new LPI(ledStrip, swerve::getPose, operator, swerve::setDesiredPose).schedule();
        } else if (Robot.gameMode == GameMode.TEST) {
            CommandScheduler.getInstance().setActiveButtonLoop(testButtonBindingLoop);
        }
        pathPlannerStorage.updatePathViewerCommand().schedule();
        this.freshCode = false;
    }

    public void updateNTGains() {
        double P = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0P")
                .getDouble(-1);
        double I = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1I")
                .getDouble(-1);
        double D = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2D")
                .getDouble(-1);

        double P2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0P")
                .getDouble(-1);
        double I2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1I")
                .getDouble(-1);
        double D2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2D")
                .getDouble(-1);

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
                && MathUtil.isNear(AutoConstants.HPFC.rotationConstants.kD, D2, 0.01))) {
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
        NamedCommands.registerCommand("StopIntake", pieceControl.stopIntakeAndIndexer());
        NamedCommands.registerCommand("StopAll", pieceControl.stopAllMotors());
        NamedCommands.registerCommand("PrepareShooter", shooterCmds.prepareFireCommandAuto(swerve::getPose));
        NamedCommands.registerCommand("Shoot", pieceControl.noteToShoot(swerve::getPose, swerve::getRobotRelativeVelocity));
        NamedCommands.registerCommand("ShootWhenReady", pieceControl.shootWhenReady(swerve::getPose, swerve::getRobotRelativeVelocity));
        NamedCommands.registerCommand("RaiseElevator", elevator.toTopCommand());
        NamedCommands.registerCommand("LowerElevator", elevator.toBottomCommand());
        NamedCommands.registerCommand("PlaceAmp", pieceControl.placeTrap());
        NamedCommands.registerCommand("PrepareShooterL", shooterCmds.prepareFireCommand(() -> FieldConstants.L_POSE, Robot::isRedAlliance));
        NamedCommands.registerCommand("PrepareShooterM", shooterCmds.prepareFireCommand(() -> FieldConstants.M_POSE, Robot::isRedAlliance));
        NamedCommands.registerCommand("PrepareShooterR", shooterCmds.prepareFireCommand(() -> FieldConstants.R_POSE, Robot::isRedAlliance));
        NamedCommands.registerCommand("PrepareShooterW3", shooterCmds.prepareFireCommand(() -> FieldConstants.W3_POSE, Robot::isRedAlliance));
        NamedCommands.registerCommand("PrepareShooter", shooterCmds.prepareFireCommand(pathPlannerStorage::getNextShotTranslation, () -> false));
        NamedCommands.registerCommand("PrepareSWD", shooterCmds.prepareSWDCommand(swerve::getPose, swerve::getRobotRelativeVelocity));
        registerPathToPathCommands();
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

        Monologue.logObj(swerve, "Robot/Swerve");

        Monologue.logObj(intake, "Robot/Subsystems/intake");
        Monologue.logObj(climb, "Robot/Subsystems/climb");
        Monologue.logObj(limelight3, "Robot/Limelights/limelight3");
        Monologue.logObj(limelight2, "Robot/Limelights/limelight2");
        Monologue.logObj(shooter, "Robot/Subsystems/shooter");
        Monologue.logObj(elevator, "Robot/Subsystems/elevator");
        Monologue.logObj(pivot, "Robot/Subsystems/pivot");
        
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
}