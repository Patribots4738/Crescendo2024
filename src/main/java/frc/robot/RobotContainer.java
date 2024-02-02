package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive;
import frc.robot.commands.ShooterCalc;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.PatriBoxController;
import frc.robot.util.PoseCalculations;
import frc.robot.util.SpeedAngleTriplet;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import monologue.Logged;

public class RobotContainer implements Logged {

    private final PatriBoxController driver;
    @SuppressWarnings("unused")
    private final PatriBoxController operator;

    private Swerve swerve;
    private final Intake intake;
    @SuppressWarnings("unused")
    private final DriverUI driverUI;

    private final Limelight limelight;
    private final Climb climb;
    private Indexer triggerWheel;

    private Shooter shooter;
    private Pivot pivot;
    private ShooterCalc shooterCalc;

    public RobotContainer() {
        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);
        DriverStation.silenceJoystickConnectionWarning(true);

        limelight = new Limelight();
        intake = new Intake();
        climb = new Climb();
        swerve = new Swerve();
        driverUI = new DriverUI();
        triggerWheel = new Indexer();
        shooter = new Shooter();
        pivot = new Pivot();
        shooterCalc = new ShooterCalc(shooter, pivot);

        limelight.setDefaultCommand(Commands.run(() -> {
            // Create an "Optional" object that contains the estimated pose of the robot
            // This can be present (sees tag) or not present (does not see tag)
            Optional<Pose2d> result = limelight.getPose2d();
            // The skew of the tag represents how confident the camera is
            // If the result of the estimatedRobotPose exists,
            // and the skew of the tag is less than 3 degrees,
            // then we can confirm that the estimated position is realistic
            if (result.isPresent()) {
                swerve.getPoseEstimator().addVisionMeasurement(
                        result.get(),
                        DriverUI.currentTimestamp - limelight.getCombinedLatencySeconds());
            }
        }, limelight));

        swerve.setDefaultCommand(new Drive(
                swerve,
                driver::getLeftY,
                driver::getLeftX,
                () -> -driver.getRightX(),
                () -> !driver.leftBumper().getAsBoolean(),
                () -> ((FieldConstants.ALLIANCE.isPresent() && FieldConstants.ALLIANCE.get().equals(Alliance.Red)))));

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

        // TODO: change the SpeedAnglePair to have the corresponding values from shooter calc
        driver.a().onTrue(shooterCalc.getNoteTrajectoryCommand(swerve::getPose, new SpeedAngleTriplet(12.0, 12.0, 45.0)));

    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto(DriverUI.autoChooser.getSelected().toString());
    }

    public void onDisabled() {
    }

    public void onEnabled() {
    }

    public void prepareNamedCommands() {
        NamedCommands.registerCommand("Intake", intake.inCommand());
    }

    private void incinerateMotors() {
        Timer.delay(0.25);
        for (CANSparkBase neo : NeoMotorConstants.motors) {
            neo.burnFlash();
            Timer.delay(0.005);
        }
        Timer.delay(0.25);
    }

    public void logPeriodicThing() {
        shooterCalc.calculateSWDAngleToSpeaker(swerve.getPose(), swerve.getRobotRelativeVelocity());
    }
}
