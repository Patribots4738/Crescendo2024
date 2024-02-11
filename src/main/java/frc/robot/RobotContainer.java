package frc.robot;

import java.util.Optional;
import com.pathplanner.lib.auto.NamedCommands;
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
        
        limelight = new Limelight();
        intake = new Intake();
        climb = new Climb();
        swerve = new Swerve();
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
            shooterCalc
        );
        
        if (limelight.isConnected()) {
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
                Robot.currentTimestamp - limelight.getCombinedLatencySeconds());
                }
            }, limelight));
        }
        
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
        // configureDriverBindings(driver);
        configureOperatorBindings(driver);
    }
    
    // TODO: uncomment these bindings (they are commented because we aren't testing them)
    private void configureOperatorBindings(PatriBoxController controller) {

        controller.povDown().onTrue(elevator.toBottomCommand());

        controller.povUp().onTrue(elevator.toTopCommand());

        controller.rightBumper()
            .onTrue(intake.outCommand());

        controller.leftBumper()
            .onTrue(intake.inCommand());

        controller.x()
            .onTrue(pieceControl.setShooterModeCommand(true));

        controller.b()
            .onTrue(pieceControl.setShooterModeCommand(false));
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

        controller.povUp().onTrue(climb.toTopCommand());
        
        controller.povDown().onTrue(climb.toBottomCommand());
        
        controller.a();
        // TODO: AMP ALIGN
        
        controller.rightTrigger();
        // TODO: SHOOT/PLACE NOTE

        controller.rightStick()
        // TODO: AIM AT SPEAKER/CHAIN
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
        NamedCommands.registerCommand("placeAmp", pieceControl.noteToTarget());
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

