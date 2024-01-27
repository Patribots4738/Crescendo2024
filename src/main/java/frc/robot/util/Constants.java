// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import java.util.Optional;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

/**
 * Welcome to the home of the many many variables :D
 * Each component of the robot gets its own class,
 * and each class has its own constants.
 * Be sure to keep it organized! And PLEASE do not forget units.
 * If possible, include them in the name of the variable,
 * or in a comment next to it.
 */
public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static double MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;

        public static final double MAX_ANGULAR_SPEED_RADS_PER_SECOND = 4 * Math.PI; // radians per second

        public static final double MAX_TELEOP_SPEED_METERS_PER_SECOND = 4.377;

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.5);
        // Distance between front and back wheels on robot
        // Easiest measured from the center of the bore of the vortex
        public static final double WHEEL_BASE = Units.inchesToMeters(21.5);

        public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(25);
        public static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(2.75);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                // Front Positive, Left Positive
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // Rear Right

        // Angular offsets of the modules relative to the chassis in radians
        // add 90 degrees to change the X and Y axis
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(180 + 90);
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(-90 + 90);
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(90 + 90);
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.toRadians(0 + 90);

        // Driving motors CAN IDs (EVEN)
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 3;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 5;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 7;

        // Turning motors CAN IDs (ODD)
        public static final int FRONT_LEFT_TURNING_CAN_ID = 4;
        public static final int REAR_LEFT_TURNING_CAN_ID = 6;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 2;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 8;

        public static final int PIGEON_CAN_ID = 29;
        public static final boolean GYRO_REVERSED = true;
    }

    public static final class ClimbConstants {

        public static final int LEFT_CLIMB_CAN_ID = 16;
        public static final int RIGHT_CLIMB_CAN_ID = 17;

        public static final double HIGH_LIMIT = 3.0;
        public static final double ROCK_BOTTOM = 0.0;
        public static final double ALMOST_HIGH_LIMIT = 2.5;
    }

    public static final class ShooterConstants {
        public static final int LEFT_SHOOTER_CAN_ID = 11;
        public static final int RIGHT_SHOOTER_CAN_ID = 12;
        public static final int SHOOTER_PIVOT_CAN_ID = 13;

        public static final double SHOOTER_P = 0.01;
        public static final double SHOOTER_I = 0;
        public static final double SHOOTER_D = 0;

        // These are in %
        public static final double SHOOTER_MIN_OUTPUT = -1;
        public static final double SHOOTER_MAX_OUTPUT = 1;

        public static final double MEASUREMENT_INTERVAL_FEET = 1.0;
        /**
         * The distances are in feet, the speeds are in RPM, and the angles are in
         * degrees.
         */
        public static final HashMap<Integer, SpeedAnglePair> SPEAKER_DISTANCES_TO_SPEEDS_AND_ANGLE_MAP = new HashMap<Integer, SpeedAnglePair>() {
            {
                put(5, SpeedAnglePair.of(0.4, 10.0));
                put(10, SpeedAnglePair.of(0.4, 10.0));
                put(15, SpeedAnglePair.of(0.4, 10.0));
                put(20, SpeedAnglePair.of(0.4, 10.0));
                put(25, SpeedAnglePair.of(0.4, 10.0));
            }
        };
    }

    public static final class TrapConstants {
        public static final int LEFT_TRAP_CAN_ID = 14;
        public static final int RIGHT_TRAP_CAN_ID = 15;
    }

    public static final class AutoConstants {

        // The below values need to be tuned for each new robot.
        // They are curently set to the values suggested by Choreo
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.377;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 7.344;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 10.468;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 37.053;

        public static final double PX_CONTROLLER = 1;
        public static final double PY_CONTROLLER = 1;
        public static final double P_THETA_CONTROLLER = 1;

        public static final double X_CORRECTION_P = 1.6;// 7;
        public static final double X_CORRECTION_I = 0;
        public static final double X_CORRECTION_D = 0;

        public static final double Y_CORRECTION_P = 1.6;// 6.03;
        public static final double Y_CORRECTION_I = 0;
        public static final double Y_CORRECTION_D = 0;

        public static final double ROTATION_CORRECTION_P = .063;
        public static final double ROTATION_CORRECTION_I = 0;
        public static final double ROTATION_CORRECTION_D = 0.00025;

        // Constraint for the motion-profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static final HolonomicDriveController HDC = new HolonomicDriveController(
                new PIDController(
                        AutoConstants.X_CORRECTION_P,
                        AutoConstants.X_CORRECTION_I,
                        AutoConstants.X_CORRECTION_D),
                new PIDController(
                        AutoConstants.Y_CORRECTION_P,
                        AutoConstants.Y_CORRECTION_I,
                        AutoConstants.Y_CORRECTION_D),
                new ProfiledPIDController(
                        AutoConstants.ROTATION_CORRECTION_P,
                        AutoConstants.ROTATION_CORRECTION_I,
                        AutoConstants.ROTATION_CORRECTION_D,
                        new TrapezoidProfile.Constraints(
                                AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)));

        public static final HolonomicPathFollowerConfig HPFC = new HolonomicPathFollowerConfig(
                new PIDConstants(
                        AutoConstants.X_CORRECTION_P,
                        AutoConstants.X_CORRECTION_I,
                        AutoConstants.X_CORRECTION_D),
                new PIDConstants(
                        AutoConstants.ROTATION_CORRECTION_P,
                        AutoConstants.ROTATION_CORRECTION_I,
                        AutoConstants.ROTATION_CORRECTION_D),
                MAX_SPEED_METERS_PER_SECOND,
                Math.hypot(DriveConstants.WHEEL_BASE, DriveConstants.TRACK_WIDTH),
                new ReplanningConfig());

    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears:
        // L1 = 12T (slow, high torque)
        // L2 = 13T (medium)
        // L3 = 14T (fast, low torque)
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a robot that drives faster).
        public static final int DRIVING_MOTOR_PINION_TEETH = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.VORTEX_FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0.0762;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_P = 0.04;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;

        public static final double TURNING_P = Robot.isSimulation() ? 0.5 : 1;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;

        public static final int NEO_CURRENT_LIMIT = 50; // amps
        public static final int VORTEX_CURRENT_LIMIT = 80; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public static final class OIConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double DRIVER_DEADBAND = 0.15;
        public static final double OPERATOR_DEADBAND = 0.15;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And
        // https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;
    }

    public static final class NeoMotorConstants {
        public static final double VORTEX_FREE_SPEED_RPM = 6784;
        public static final double NEO_FREE_SPEED_RPM = 5676;

        public static final int MAX_PERIODIC_STATUS_TIME_MS = 65535;
        public static final int FAST_PERIODIC_STATUS_TIME_MS = 10;
      
        public static ArrayList<Neo> motors = new ArrayList<>();
    }

    public static final class IntakeConstants {
        public static final int INTAKE_CAN_ID = 10;
        public static final int TRIGGER_WHEEL_CAN_ID = 11;

        // % speeds of the motor
        public static final double INTAKE_SPEED = 0.5;
        public static final double OUTTAKE_SPEED = -0.5;
        public static final double STOP_SPEED = 0;

        public static final int INTAKE_FREE_CURRENT_LIMIT_AMPS = 15;
        public static final int INTAKE_STALL_CURRENT_LIMIT_AMPS = 7;

        public static final int HAS_PIECE_CURRENT_THRESHOLD = 20;
    }

    public static final class FieldConstants {

        public static boolean IS_SIMULATION = Robot.isSimulation();

        public static final double ALIGNMENT_SPEED = 3;
        public static final double SNAP_TO_ANGLE_P = 0.0025;

        public static final double ALLOWABLE_ERROR_METERS = Units.inchesToMeters(2);
        public static final double FIELD_WIDTH_METERS = 16.5410515;
        public static final double FIELD_HEIGHT_METERS = 8.2112312;

        public static Optional<Alliance> ALLIANCE = Optional.empty();

        public static enum GameMode {
            DISABLED,
            AUTONOMOUS,
            TELEOP,
            TEST
        };

        public static GameMode GAME_MODE;

        // Field:
        // https://cad.onshape.com/documents/dcbe49ce579f6342435bc298/w/b93673f5b2ec9c9bdcfec487/e/6ecb2d6b7590f4d1c820d5e3
        // Chain Positions: Blue alliance left
        // @formatter:off
        //  2              5
        //      1      4
        //  0              3
        // @formatter:on
        public static Pose2d[] CHAIN_POSITIONS = new Pose2d[] {
                // All points are in meters and radians
                // All relative to the blue origin
                // Blue Stage
                new Pose2d(4.37, 3.201, Rotation2d.fromDegrees(60)),
                new Pose2d(5.875, 4.168, Rotation2d.fromDegrees(180)),
                new Pose2d(4.353, 4.938, Rotation2d.fromDegrees(-60)),
                // Red Stage
                new Pose2d(12.07, 3.237, Rotation2d.fromDegrees(120)),
                new Pose2d(10.638, 4.204, Rotation2d.fromDegrees(0)),
                new Pose2d(12.2, 5, Rotation2d.fromDegrees(-120))
        };

        // Speaker Positions: Blue alliance left
        // @formatter:off
        //
        //  0             1
        //
        //
        // @formatter:on
        public static Pose2d[] SPEAKER_POSITIONS = new Pose2d[] {
                // All points are in meters and radians
                // All relative to the blue origin
                // Blue Speaker
                new Pose2d(0, 5.547, Rotation2d.fromDegrees(0)),
                // Red Speaker
                new Pose2d(FIELD_WIDTH_METERS, 5.547, Rotation2d.fromDegrees(180)),
        };

        public static Pose2d[] AMP_POSITIONS = new Pose2d[] {
                // All points are in meters and radians
                // All relative to the blue origin
                // Blue Speaker
                new Pose2d(1.827, FIELD_HEIGHT_METERS, Rotation2d.fromDegrees(-90)),
                // Red Speaker
                new Pose2d(14.706, FIELD_HEIGHT_METERS, Rotation2d.fromDegrees(-90)),
        };

        public static final double CHAIN_LENGTH_METERS = Units.inchesToMeters(100);

        public static enum ChainPosition {
            LEFT,
            RIGHT,
            CENTER
        }
    }
}