// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Pose3dLogger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriverUI;
import frc.robot.commands.Drive;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Swerve extends SubsystemBase implements Logged {

    public static double twistScalar = 4;
    
    private double speedMultiplier = 1;

    private final MAXSwerveModule frontLeft = new MAXSwerveModule(
            DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
            DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule frontRight = new MAXSwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
            DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearLeft = new MAXSwerveModule(
            DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
            DriveConstants.REAR_LEFT_TURNING_CAN_ID,
            DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearRight = new MAXSwerveModule(
            DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
            DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
            DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private double[] desiredModuleStates = new double[6];
    private double[] realModuleStates = new double[6];

    @Log.NT
    Pose3d robotPose3d = new Pose3d();

    // The gyro sensor
    private final Pigeon2 gyro = new Pigeon2(DriveConstants.GYRO_CAN_ID);

    private final MAXSwerveModule[] swerveModules = new MAXSwerveModule[] {
            frontLeft,
            frontRight,
            rearLeft,
            rearRight
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            gyro.getRotation2d(),
            getModulePositions(),
            new Pose2d(),
            // Trust the information of the vision more
            // Nat.N1()).fill(0.1, 0.1, 0.1) --> trust more
            // Nat.N1()).fill(1.25, 1.25, 1.25) --> trust less
            // Notice that the theta on the vision is very large,
            // and the state measurement is very small.
            // This is because we assume that the IMU is very accurate.
            // You can visualize these graphs working together here:
            // https://www.desmos.com/calculator/a0kszyrwfe
            VecBuilder.fill(0.1, 0.1, 0.05),
            // State measurement
            // standard deviations
            // X, Y, theta
            VecBuilder.fill(0.6, 0.6, 2)
    // Vision measurement
    // standard deviations
    // X, Y, theta
    );

    /**
     * Creates a new DriveSu1stem.
     */
    public Swerve() {
        configurateMotors();

        resetEncoders();
        gyro.setYaw(0);
        setBrakeMode();

        realModuleStates = new double[] { 0, 0, 0, 0, 0, 0, 0, 0 };

        SmartDashboard.putNumberArray("Swerve/RealStates", realModuleStates);
        SmartDashboard.putNumberArray("Swerve/DesiredStates", desiredModuleStates);
        SmartDashboard.putNumber("Swerve/RobotRotation", getPose().getRotation().getDegrees());
        
    }

    @Override
    public void periodic() {

        poseEstimator.updateWithTime(DriverUI.currentTimestamp, gyro.getRotation2d(), getModulePositions());
        logPositions();

        if (FieldConstants.IS_SIMULATION) {
            for (MAXSwerveModule mod : swerveModules) {
                mod.tick();
            }
        }

        SwerveModuleState[] measuredStates = new SwerveModuleState[] {
            frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
        };
        
        ChassisSpeeds speeds = DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(measuredStates);
        resetOdometry(
            getPose().exp(
                new Twist2d(
                    0, 0,
                    speeds.omegaRadiansPerSecond * .02)));

        realModuleStates = new double[] {
            measuredStates[0].angle.getRadians(), measuredStates[0].speedMetersPerSecond,
            measuredStates[1].angle.getRadians(), measuredStates[1].speedMetersPerSecond,
            measuredStates[2].angle.getRadians(), measuredStates[2].speedMetersPerSecond,
            measuredStates[3].angle.getRadians(), measuredStates[3].speedMetersPerSecond
        };
    }

    public void logPositions() {
        DriverUI.field.setRobotPose(getPose());

        SmartDashboard.putNumberArray("Swerve/RealStates", realModuleStates);

        SmartDashboard.putNumberArray("Swerve/DesiredStates", desiredModuleStates);
        SmartDashboard.putNumber("Swerve/RobotRotation", gyro.getRotation2d().getRadians());
        
        robotPose3d = new Pose3d(
                                new Translation3d(
                                        getPose().getX(),
                                        getPose().getY(),
                                        Math.hypot(
                                                Rotation2d.fromDegrees(gyro.getRoll().refresh().getValue()).getSin()
                                                        * DriveConstants.ROBOT_LENGTH_METERS / 2.0,
                                                Rotation2d.fromDegrees(gyro.getPitch().refresh().getValue()).getSin() *
                                                        DriveConstants.ROBOT_LENGTH_METERS / 2.0)),
                                gyro.getRotation3d());

        SmartDashboard.putNumberArray("RobotPose3d",
                Pose3dLogger.composePose3ds(robotPose3d));
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {

        xSpeed *= (DriveConstants.MAX_SPEED_METERS_PER_SECOND * speedMultiplier);
        ySpeed *= (DriveConstants.MAX_SPEED_METERS_PER_SECOND * speedMultiplier);
        rotSpeed *= (DriveConstants.DYNAMIC_MAX_ANGULAR_SPEED * speedMultiplier);

        SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                                getPose().getRotation()), (Timer.getFPGATimestamp()-DriverUI.previousTimestmap))
                        : ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed), (Timer.getFPGATimestamp()-DriverUI.previousTimestmap)));

        setModuleStates(swerveModuleStates);

    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setWheelsX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    }

    public Command getSetWheelsX() {
        return run(this::setWheelsX);
    }

    public void setWheelsUp() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(gyro.getRotation2d())));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(gyro.getRotation2d())));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(gyro.getRotation2d())));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90).minus(gyro.getRotation2d())));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);

        desiredModuleStates = new double[] {
                desiredStates[0].angle.getRadians(), desiredStates[0].speedMetersPerSecond,
                desiredStates[1].angle.getRadians(), desiredStates[1].speedMetersPerSecond,
                desiredStates[2].angle.getRadians(), desiredStates[2].speedMetersPerSecond,
                desiredStates[3].angle.getRadians(), desiredStates[3].speedMetersPerSecond
        };
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                gyro.getRotation2d(),
                getModulePositions(),
                pose);
    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int modNum = 0; modNum < swerveModules.length; modNum++) {
            states[modNum] = swerveModules[modNum].getState();
        }
        return states;

    }

    public double getSpeedMetersPerSecond() {
        // We update the UI at the end of the loop, so this is a way of looking into the
        // past.
        return ((DriverUI.field.getRobotPose().getTranslation().minus(getPose().getTranslation()).getNorm()) / 0.02);
    }

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

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public Command toggleSpeed() {
        return runOnce(() -> this.speedMultiplier = (this.speedMultiplier == 1) ? 0.35 : 1);
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public Command setAlignemntSpeed() {
        return runOnce(() -> {
                DriveConstants.MAX_SPEED_METERS_PER_SECOND = FieldConstants.ALIGNMENT_SPEED;
        });
    }

    public double getSpeedMultiplier() {
        return this.speedMultiplier;
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
    
    public Command getAutoAlignmentCommand(Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds) {
      return new Drive(this, () -> {
        ChassisSpeeds controllerSpeedsGet = controllerSpeeds.get();
        ChassisSpeeds autoSpeedsGet = autoSpeeds.get();
        return new ChassisSpeeds(
            controllerSpeedsGet.vxMetersPerSecond + autoSpeedsGet.vxMetersPerSecond,
            controllerSpeedsGet.vyMetersPerSecond + autoSpeedsGet.vyMetersPerSecond,
            controllerSpeedsGet.omegaRadiansPerSecond + autoSpeedsGet.omegaRadiansPerSecond
        );
    }, () -> false, () -> false);
    }

    public Command getDriveCommand(Supplier<ChassisSpeeds> speeds, boolean fieldRelative, boolean rateLimit) {
      return new Drive(this, speeds, () -> fieldRelative, () -> false);
    }

    public void configurateMotors() {
        for(int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].getConfigCommand().ignoringDisable(true).schedule();
            Timer.delay(0.25);
        }
    }
}
