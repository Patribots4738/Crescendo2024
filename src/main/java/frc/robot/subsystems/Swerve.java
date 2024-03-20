// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import java.util.Arrays;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.ChasePose;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.DriveHDC;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.rev.MAXSwerveModule;
import monologue.Logged;
import monologue.Annotations.Log;

public class Swerve extends SubsystemBase implements Logged {

    public static double twistScalar = 4;
    private Rotation2d gyroRotation2d = new Rotation2d();

    @Log
    private boolean isAlignedToAmp = false;

    private final MAXSwerveModule frontLeft, frontRight, rearLeft, rearRight;

    // The gyro sensor
    private final Pigeon2 gyro;

    private final MAXSwerveModule[] swerveModules;

    private SwerveDrivePoseEstimator poseEstimator;

    /**
     * Creates a new DriveSu1stem.
     */
    public Swerve() {

        frontLeft = new MAXSwerveModule(
            DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
            DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

        frontRight = new MAXSwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
            DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

        rearLeft = new MAXSwerveModule(
            DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
            DriveConstants.REAR_LEFT_TURNING_CAN_ID,
            DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

        rearRight = new MAXSwerveModule(
            DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
            DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
            DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);     

        swerveModules = new MAXSwerveModule[] {
            frontLeft,
            frontRight,
            rearLeft,
            rearRight
        };
            
        gyro = new Pigeon2(DriveConstants.PIGEON_CAN_ID);
        gyro.setYaw(0);
        resetEncoders();
        setBrakeMode();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeVelocity,
                this::drive,
                AutoConstants.HPFC,
                Robot::isRedAlliance,
                this);

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            gyro.getRotation2d(),
            getModulePositions(),
            new Pose2d(),
            // State measurements
            /*
             * See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
             * for how to select the standard deviations.
             */
            // standard deviations
            // X, Y, theta
            VecBuilder.fill(
                0.003, // 6328 uses 0.003 m here
                0.003, // 6328 uses 0.003 m here
                0.0002 // 6328 uses 0.0002 rads here
            ),
            // Vision measurement
            // standard deviations
            // X, Y, theta
            VecBuilder.fill(
                0.192,
                0.192,
                Units.degreesToRadians(15)
            )
        );
    }

    @Override
    public void periodic() {
        gyroRotation2d = gyro.getRotation2d();
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyroRotation2d, getModulePositions());
        
        logPositions();
        this.isAlignedToAmp = isAlignedToAmp();
    }

    @Log
    Pose2d currentPose = new Pose2d();

    public void logPositions() {

        currentPose = getPose();

        RobotContainer.swerveMeasuredStates = new SwerveModuleState[] {
                frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
        };

        ChassisSpeeds speeds = DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(RobotContainer.swerveMeasuredStates);

        if (FieldConstants.IS_SIMULATION) {
            resetOdometry(
                    currentPose.exp(
                            new Twist2d(
                                    0, 0,
                                    speeds.omegaRadiansPerSecond * .02)));
        }

        RobotContainer.field2d.setRobotPose(currentPose);

        if ((Double.isNaN(currentPose.getX())
            || Double.isNaN(currentPose.getY())
            || Double.isNaN(currentPose.getRotation().getDegrees())))
        {
            // Something in our pose was NaN...
            resetOdometry(RobotContainer.robotPose2d);
            resetEncoders();
            resetHDC();
        } else {
            RobotContainer.robotPose2d = currentPose;
        }

        RobotContainer.robotPose3d = new Pose3d(
                new Translation3d(
                        currentPose.getX(),
                        currentPose.getY(),
                        Math.hypot(
                                Rotation2d.fromDegrees(gyro.getRoll().refresh().getValue()).getSin()
                                        * DriveConstants.ROBOT_LENGTH_METERS / 2.0,
                                Rotation2d.fromDegrees(gyro.getPitch().refresh().getValue()).getSin() *
                                        DriveConstants.ROBOT_LENGTH_METERS / 2.0)),
                new Rotation3d(0, 0, currentPose.getRotation().getRadians()));

        RobotContainer.distanceToSpeakerMeters = currentPose.getTranslation().getDistance(FieldConstants.GET_SPEAKER_TRANSLATION());

    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getGyroRotation2d() {
        return this.gyroRotation2d;
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void drive(ChassisSpeeds robotRelativeSpeeds) {
        setModuleStates(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.discretize(robotRelativeSpeeds, (Timer.getFPGATimestamp() - Robot.previousTimestamp)))
        );
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        double timeDifference = Timer.getFPGATimestamp() - Robot.previousTimestamp;
        ChassisSpeeds robotRelativeSpeeds;

        if (fieldRelative) {
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getPose().getRotation());
        } else {
            robotRelativeSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, timeDifference);
        SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(discretizedSpeeds);

        setModuleStates(swerveModuleStates);
    }

    public void stopDriving() {
        drive(0, 0, 0, false);
    }
    
    @Log
    Pose2d desiredHDCPose = new Pose2d();
    public void setDesiredPose(Pose2d pose) {
        desiredHDCPose = pose;
    }

    public ChassisSpeeds getRobotRelativeVelocity() {
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setWheelsX() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };

        setModuleStates(desiredStates);
    }


    public Command getSetWheelsX() {
        return run(this::setWheelsX);
    }   

    public void setWheelsO() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };

        setModuleStates(desiredStates);
    }

    public Command setWheelsOCommand() {
        return runOnce(this::setWheelsO);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, 
            DriveConstants.MAX_SPEED_METERS_PER_SECOND
        );
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);

        RobotContainer.swerveDesiredStates = desiredStates;
    }

    public void resetOdometry(Pose2d pose) {

        if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getRotation().getRadians())) {
            return;
        }

        poseEstimator.resetPosition(
                gyroRotation2d,
                getModulePositions(),
                pose);
    }

    public Command resetOdometryCommand(Supplier<Pose2d> pose) {
        return runOnce(() -> resetOdometry(pose.get())).ignoringDisable(true);
    }

    public Command resetPositionCommand(Supplier<Translation2d> pose) {
        return runOnce(() -> resetOdometry(new Pose2d(pose.get(), getPose().getRotation()))).ignoringDisable(true);
    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            states[modNum] = swerveModules[modNum].getState();
        }
        return states;

    }

    /**
     * Returns an array of SwerveModulePosition objects representing the positions of all swerve modules.
     * This is the position of the driving encoder and the turning encoder
     *
     * @return an array of SwerveModulePosition objects representing the positions of all swerve modules
     */
    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            positions[modNum] = swerveModules[modNum].getPosition();
        }
        return positions;

    }
    
    public void resetEncoders() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.resetEncoders();
        }
    }

    public double[] getWheelRadiusCharacterizationPosition() {
        return Arrays.stream(swerveModules).mapToDouble(module -> module.getDrivePositionRadians()).toArray();
    }

    /**
     * Sets the brake mode for the drive motors.
     * This is useful for when the robot is enabled
     * So we can stop the robot quickly
     * (This is the default mode)
     */
    public void setBrakeMode() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.setBrakeMode();
        }
    }

    /**
     * Sets the coast mode for the drive motors.
     * This is useful for when the robot is disabled
     * So we can freely move the robot around
     */
    public void setCoastMode() {
        for (MAXSwerveModule mSwerveMod : swerveModules) {
            mSwerveMod.setCoastMode();
        }
    }

    public Command getDriveCommand(Supplier<ChassisSpeeds> speeds, BooleanSupplier fieldRelative) {
        return new Drive(this, speeds, fieldRelative, () -> false);
    }
    
    public DriveHDC getDriveHDCCommand(Supplier<ChassisSpeeds> speeds, BooleanSupplier fieldRelative) {
        return new DriveHDC(this, speeds, fieldRelative, () -> false);
    }

    public Command getChaseCommand(Supplier<Pose2d> desiredPose, BooleanSupplier cancelSupplier) {
        return new ChasePose(this, desiredPose, cancelSupplier);
    }

    public Command getChaseCommand(Supplier<Pose2d> desiredPose) {
        return new ChasePose(this, desiredPose, () -> false);
    }

    public Command getScanCommand() {
        return new ChasePose(this, () ->
            new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(Robot.isRedAlliance() ? 0 : 180)).plus(
                new Transform2d(
                    0,
                    0, 
                    Rotation2d.fromRadians(
                        Math.cos(Robot.currentTimestamp)/2.0)
                    )
                ), 
                () -> false
        );
    }

    public void resetHDC() {
        AutoConstants.HDC.getThetaController().reset(getPose().getRotation().getRadians());
    }

    public Command resetHDCCommand() {
        return Commands.runOnce(() -> resetHDC());
    }

    public void reconfigureAutoBuilder() {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeVelocity,
                this::drive,
                AutoConstants.HPFC,
                Robot::isRedAlliance,
                this);
    }

    // Used for note pickup in auto
    // Because of the fact that we do not have to be perfectly 
    // on top of a note to intake it, the tolerance is fairly lenient
    public boolean atPose(Pose2d position) {
        // More lenient on x axis, less lenient on y axis and rotation
        Pose2d currentPose = getPose();
        double angleDiff = currentPose.getRotation().minus(position.getRotation()).getRadians();
		double distance = currentPose.relativeTo(position).getTranslation().getNorm();
        return 
            MathUtil.isNear(0, distance, AutoConstants.AUTO_POSITION_TOLERANCE_METERS)
            && MathUtil.isNear(0, angleDiff, AutoConstants.AUTO_POSITION_TOLERANCE_RADIANS);
    }

    public boolean atHDCPose() {
        return atPose(desiredHDCPose);
    }

    public boolean isAlignedToAmp() {
        Translation2d touchingAmpPose = new Translation2d(
            FieldConstants.GET_AMP_POSITION().getX(),
            FieldConstants.GET_AMP_POSITION().getY() 
                - DriveConstants.ROBOT_LENGTH_METERS / 2.0
                - DriveConstants.BUMPER_LENGTH_METERS
        );
        double robotX = this.getPose().getTranslation().getDistance(touchingAmpPose);
        return MathUtil.isNear(0, robotX, AutoConstants.AUTO_ALIGNMENT_DEADBAND);
    }
}