package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive;
import frc.robot.commands.PieceControl;
import frc.robot.commands.ShooterCalc;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightHelpers.Results;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.PatriBoxController;
import frc.robot.util.PoseCalculations;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class RobotContainer implements Logged {

    private final PatriBoxController driver;
    private final PatriBoxController operator;

    private Swerve swerve;
    private final Intake intake;

    @SuppressWarnings("unused")
    private final DriverUI driverUI;
    private final Limelight limelight;
    private final Climb climb;
    private Indexer triggerWheel;
    private Pivot pivot;
    private Shooter shooter;
    private Claw claw;
    private Elevator elevator;
    private ShooterCalc shooterCalc;
    private PieceControl pieceControl;

    @Log.NT
    public static Pose3d[] components3d = new Pose3d[5];
    @Log.NT
    public static Pose3d[] desiredComponents3d = new Pose3d[5];

    public RobotContainer() {
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);
        DriverStation.silenceJoystickConnectionWarning(true);

        intake = new Intake();
        climb = new Climb();
        swerve = new Swerve();
        driverUI = new DriverUI();
        triggerWheel = new Indexer();
        limelight = new Limelight(swerve::getPose);

        shooter = new Shooter();
        elevator = new Elevator();
        claw = new Claw();

        pivot = new Pivot();

        shooterCalc = new ShooterCalc(shooter, pivot);

        pieceControl = new PieceControl(
                intake,
                triggerWheel,
                elevator,
                claw,
                shooterCalc,
                swerve);

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
                        DriverUI.currentTimestamp - limelight.getLatencyDiffSeconds());
            }
        }, limelight));

        swerve.setDefaultCommand(new Drive(
                swerve,
                driver::getLeftY,
                driver::getLeftX,
                () -> -driver.getRightX(),
                () -> !driver.leftBumper().getAsBoolean(),
                () -> (driver.leftBumper().getAsBoolean()
                        && FieldConstants.ALLIANCE.equals(Optional.ofNullable(Alliance.Blue)))));

        incinerateMotors();
        configureButtonBindings();

        prepareNamedCommands();
    }

    private void configureButtonBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureOperatorBindings() {
        operator.y().onTrue((climb.toTop(PoseCalculations.getChainPosition(swerve.getPose()))));

        operator.a().onTrue((climb.toBottom()));

        operator.b().onTrue(
                pieceControl.prepareToFire(operator.leftBumper()));

        operator.leftBumper().and(operator.rightBumper().negate()).onTrue(
                pieceControl.prepareToFire(operator.x()));

        operator.leftBumper().and(operator.rightBumper()).onTrue(
                pieceControl.noteToShoot());

        operator.rightBumper().and(operator.leftBumper().negate()).onTrue(
                pieceControl.noteToTarget(() -> true));

        operator.leftTrigger(OIConstants.OPERATOR_DEADBAND).and(
                intake.hasGamePieceTrigger().negate()).onTrue(
                        intake.inCommand());

        operator.rightTrigger(OIConstants.OPERATOR_DEADBAND).onTrue(
                intake.outCommand());

        operator.x().onTrue(
                intake.stop());
    }

    private void configureDriverBindings() {

        // Upon hitting start or back,
        // reset the orientation of the robot
        // to be facing away from the driver station
        driver.start().or(driver.back()).onTrue(
                Commands.runOnce(() -> swerve.resetOdometry(
                        new Pose2d(
                                swerve.getPose().getTranslation(),
                                Rotation2d.fromDegrees(
                                        FieldConstants.ALLIANCE.equals(Optional.of(Alliance.Red))
                                                ? 0
                                                : 180))),
                        swerve));

        driver.rightBumper().whileTrue(Commands.runOnce(swerve::getSetWheelsX));

        driver.leftStick().toggleOnTrue(swerve.toggleSpeed());

        driver.a().and(intake.hasGamePieceTrigger().negate()).onTrue(intake.inCommand());

        driver.y().onTrue(intake.outCommand());

        driver.x().onTrue(intake.stop());

        driver.rightStick().whileTrue(
                Commands.sequence(
                        swerve.getDriveCommand(
                                () -> {
                                    return ChassisSpeeds.fromFieldRelativeSpeeds(
                                            driver.getLeftY(),
                                            driver.getLeftX(),
                                            swerve.getAlignmentSpeeds(Rotation2d.fromRadians(360)),
                                            swerve.getPose().getRotation());
                                }, true)));

    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto(DriverUI.autoChooser.getSelected().toString());
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
        NamedCommands.registerCommand("prepareShooterL",
                shooterCalc.prepareFireCommand(() -> true, FieldConstants.L_POSE));
        NamedCommands.registerCommand("prepareShooterM",
                shooterCalc.prepareFireCommand(() -> true, FieldConstants.M_POSE));
        NamedCommands.registerCommand("prepareShooterR",
                shooterCalc.prepareFireCommand(() -> true, FieldConstants.R_POSE));
    }

    private void incinerateMotors() {
        Timer.delay(0.25);
        for (CANSparkBase neo : NeoMotorConstants.motors) {
            neo.burnFlash();
            Timer.delay(0.005);
        }
        Timer.delay(0.25);
    }

}
