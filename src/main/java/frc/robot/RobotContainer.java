package frc.robot;

import java.util.Optional;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.util.PatriBoxController;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.PatriBoxController;
import frc.robot.util.PoseCalculations;
import frc.robot.util.SpeedAngleTriplet;
import monologue.Annotations.Log;
import monologue.Logged;
import frc.robot.util.Constants.NTConstants;
import monologue.Annotations.Log;

public class RobotContainer implements Logged {
    
    private final PatriBoxController driver;
    private final PatriBoxController operator;

    private Swerve swerve;
    private final Intake intake;

    @SuppressWarnings("unused")
    private final DriverUI driverUI;
    private final Limelight limelight;
    private final LedStrip ledStrip;
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
        
        limelight = new Limelight();
        intake = new Intake();
        climb = new Climb();
        swerve = new Swerve();
        driverUI = new DriverUI();
        ledStrip = new LedStrip(swerve::getPose);
        triggerWheel = new Indexer();
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
            () -> (driver.leftBumper().getAsBoolean()
                && FieldConstants.IS_BLUE_ALLIANCE())));
              
        incinerateMotors();
        configureButtonBindings();
        
        prepareNamedCommands();
        initializeArrays();
    }
    
    private void configureButtonBindings() {
        configureDriverBindings(driver);
        configureOperatorBindings(operator);
    }
    
    private void configureOperatorBindings(PatriBoxController controller) {

        controller.povUp().toggleOnTrue(climb.povUpCommand(swerve::getPose));
        
        controller.povDown().onTrue(climb.toBottomCommand());

        controller.povLeft().onTrue(elevator.toBottomCommand());

        controller.povRight().onTrue(pieceControl.placeTrapCommand());

        controller.leftBumper()
            .and(controller.rightBumper())
            .onTrue(pieceControl.noteToShoot());

        controller.rightBumper()
            .and(controller.leftBumper().negate())
            .onTrue(pieceControl.noteToTarget(() -> true));

        controller.leftTrigger(OIConstants.OPERATOR_DEADBAND)
            .and(intake.hasGamePieceTrigger().negate())
            .onTrue(intake.inCommand());

        controller.rightTrigger(OIConstants.OPERATOR_DEADBAND)
            .onTrue(intake.outCommand());

        controller.x().onTrue(intake.stop());
    }
    
    private void configureDriverBindings(PatriBoxController controller) {
        
        // Upon hitting start or back,
        // reset the orientation of the robot
        // to be facing away from the driver station
        controller.start().or(driver.back()).onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(),
                    Rotation2d.fromDegrees(
                        FieldConstants.IS_RED_ALLIANCE()
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
            .toggleOnTrue(shooterCalc.prepareFireMovingCommand(() -> true, swerve::getPose));
        
        controller.leftTrigger()
            .onTrue(shooterCalc.resetShooter());
        
        controller.x()
            .onTrue(intake.stop());
        
        controller.rightStick()
            .whileTrue(
                Commands.sequence(
                swerve.resetHDC(),
                swerve.getDriveCommand(
                    () -> {
                        return new ChassisSpeeds(
                            controller.getLeftY(),
                            controller.getLeftX(),
                            swerve.getAlignmentSpeeds(Rotation2d.fromDegrees(270)));
                    },
                    () -> true)));
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
    }
}

