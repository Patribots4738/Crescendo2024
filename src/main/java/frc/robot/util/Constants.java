package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import java.util.Optional;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;

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

        // Front positive, left positive
        public static final Translation2d FRONT_LEFT_WHEEL_POSITION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_WHEEL_POSITION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d REAR_LEFT_WHEEL_POSITION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d REAR_RIGHT_WHEEL_POSITION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

        public static final Translation2d[] WHEEL_POSITION_ARRAY = new Translation2d[] {
            FRONT_LEFT_WHEEL_POSITION,
            FRONT_RIGHT_WHEEL_POSITION,
            REAR_LEFT_WHEEL_POSITION,
            REAR_RIGHT_WHEEL_POSITION
        };

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                WHEEL_POSITION_ARRAY
        );

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


    public static final class TuningConstants implements Logged {
        public final int DRIVE_INDEX = 0;
        public final int PIVOT_INDEX = 1;
        public final int SHOOTER_INDEX = 2;
        public final int ELEVATOR_INDEX = 3;
        public final int CLIMB_INDEX = 4;

    }

    public static final class ShooterConstants {
        public static final int LEFT_SHOOTER_CAN_ID = 11;
        public static final int RIGHT_SHOOTER_CAN_ID = 12;
        public static final int SHOOTER_PIVOT_CAN_ID = 13;

        public static final double SHOOTER_VELOCITY_CONVERSION_FACTOR = 1.0;
        // degrees
        public static final double PIVOT_POSITION_CONVERSION_FACTOR = 360;

        public static final PIDConstants SHOOTER_PID = new PIDConstants(
            0.002,
            0,
            0.20992
        );
        public static final double SHOOTER_FF = 0.0001762;

        public static final PIDConstants PIVOT_PID = new PIDConstants(
            0.05,
            0,
            0.0083
        );

        public static final int SHOOTER_CURRENT_LIMIT = 80;
        public static final int PIVOT_CURRENT_LIMIT = 15;

        public static final double SHOOTER_BACK_SPEED = -0.5;

        public static final double PIVOT_DEADBAND = .5;
        public static final double SHOOTER_DEADBAND = 0.3;

        // These are in %
        public static final double SHOOTER_MIN_OUTPUT = -1;
        public static final double SHOOTER_MAX_OUTPUT = 1;

        public static final double PIVOT_MIN_OUTPUT = -0.25;
        public static final double PIVOT_MAX_OUTPUT = 0.25;

        public static final double PIVOT_LOWER_LIMIT_DEGREES_WRONG = 343.5;
        public static final double PIVOT_UPPER_LIMIT_DEGREES_WRONG = 300;

        public static final double PIVOT_LOWER_LIMIT_DEGREES = 17;
        public static final double PIVOT_UPPER_LIMIT_DEGREES = 60;

        public static final double SHOOTER_RPM_LOWER_LIMIT = -NeoMotorConstants.NEO_FREE_SPEED_RPM;
        public static final double SHOOTER_RPM_UPPER_LIMIT = NeoMotorConstants.NEO_FREE_SPEED_RPM;

        public static final double SHOOTER_PASS_SECONDS = 2;

        public static final double MEASUREMENT_INTERVAL_FEET = 1.0;
        /**
         * The distances are in feet, the speeds are in RPM, and the angles are in
         * degrees.
         * The distances are in feet, the speeds are in RPM, and the angles are in
         * degrees.
         */
        public static final HashMap<Integer, SpeedAngleTriplet> SPEAKER_DISTANCES_TO_SPEEDS_AND_ANGLE_MAP = new HashMap<Integer, SpeedAngleTriplet>() {
            {
                put(5, SpeedAngleTriplet.of(3000.0, 3000.0, 45.0));
                put(10, SpeedAngleTriplet.of(4000.0, 4000.0, 45.0));
                put(15, SpeedAngleTriplet.of(6000.0, 4000.0, 40.0));
                put(20, SpeedAngleTriplet.of(6000.0, 4000.0, 30.0));
                put(25, SpeedAngleTriplet.of(6000.0, 4000.0, 20.0));
                put(30, SpeedAngleTriplet.of(6000.0, 4000.0, 10.0));
            }
        };

        public static final InterpolatingTreeMap<Double, SpeedAngleTriplet> INTERPOLATION_MAP = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(),
                SpeedAngleTriplet.getInterpolator()) {{
            for (Map.Entry<Integer, SpeedAngleTriplet> entry : SPEAKER_DISTANCES_TO_SPEEDS_AND_ANGLE_MAP.entrySet()) {
                put(entry.getKey().doubleValue(), entry.getValue());
            }
        }};

        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);

    }

    public static final class TrapConstants {
        public static final int ELEVATOR_CAN_ID = 14;
        public static final int CLAW_CAN_ID = 15;
        public static final double ELEVATOR_DEADBAND = .003;
        public static final double OUTTAKE_SECONDS = 1;
        public static final double CLAW_POSITION_MULTIPLIER = 1.83;

        public static final int CLAW_CURRENT_LIMIT = 15;

        public static final double TRAP_ELEVATOR_MAX_OUTPUT = 1;
        public static final double TRAP_ELEVATOR_MIN_OUTPUT = -TRAP_ELEVATOR_MAX_OUTPUT;

        public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.0 / 25.0;

        public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 20; // amps

        public static final PIDConstants TRAP_PID = new PIDConstants(0.5, 0, 0);

        // TODO: set these values
        public static final double RESET_POS = 0;
        public static final double INTAKE_TIME = 0;
        public static final double CLAW_OUTTAKE_PERCENT = -1;
        public static final double CLAW_INTAKE_PERCENT = 1;
        public static final double CLAW_STOP_PERCENT = 0;
        public static final double TRAP_PLACE_POS = 0.48;
        public static final double AMP_PLACE_POS = 0.48;

        public static final double ELEVATOR_TOP_LIMIT = 0.48;
        public static final double ELEVATOR_BOTTOM_LIMIT = 0;

        public static final double CLAW_LOWER_PERCENT_LIMIT = 1;
        public static final double CLAW_UPPER_PERCENT_LIMIT = -1;

        public static final double CLAW_HAS_PIECE_UPPER_LIMIT = 0;
        public static final double CLAW_HAS_PIECE_LOWER_LIMIT = -0.25;

        public static final double CLAW_HAS_PIECE_MIN_TIMESTAMP = 0.25;

        public static final double CLAW_HAS_PIECE_MIN_CURRENT = 15;

        public static final double CLAW_HAS_PIECE_MIN_TARGET_VELO = 0.45;
    }

    public static final class ClimbConstants {

        public static final int  LEFT_CLIMB_CAN_ID = 16;
        public static final int RIGHT_CLIMB_CAN_ID = 17;

        private static final double GEAR_RATIO = 16.0/Units.inchesToMeters(3.0);
        private static final double CLIMB_HEIGHT = Units.inchesToMeters(20)*2.0;
        
        public static final double CLIMB_POSITION_CONVERSION_FACTOR = 1.0/(GEAR_RATIO*CLIMB_HEIGHT);
        public static final int CLIMB_CURRENT_LIMIT = 40;

        public static final PIDConstants CLIMB_PID = new PIDConstants(5, 0, 0);

        public static final double EXTENSION_LIMIT_METERS = Units.feetToMeters(3.65);
        
        public static final double TOP_LIMIT = 0.517;
        public static final double BOTTOM_LIMIT = 0.0;

        public static final double CLIMB_DEADBAND = 0.002;
        
        public static final double DISTANCE_FROM_ORIGIN_METERS = 0.3048;
    }

    public static final class AutoConstants {

        // The below values need to be tuned for each new robot.
        // They are currently set to the values suggested by Choreo
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

        public static final double ROTATION_CORRECTION_P = .63;
        public static final double ROTATION_CORRECTION_I = 0;
        public static final double ROTATION_CORRECTION_D = 0.0025;

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
        public static final int PID_TUNER_CONTROLLER_PORT = 2;

        public static final double DRIVER_DEADBAND = 0.15;
        public static final double OPERATOR_DEADBAND = 0.15;
        public static final double PID_TUNER_DEADBAND = 0.15;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And
        // https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;
    }

    public static final class LEDConstants {
        public static final int PWM_PORT = 9;
        public static final int LED_COUNT = new AddressableLEDBuffer(PWM_PORT).getLength();

        public static final Pose2d[] startingPositions = new Pose2d[] {
            new Pose2d(),
            new Pose2d(1, 1, new Rotation2d(Units.degreesToRadians(120))),
            new Pose2d(2, 1, Rotation2d.fromDegrees(10)),
            new Pose2d(3, 1, Rotation2d.fromRadians(Math.PI)),
            new Pose2d(new Translation2d(1, 2), new Rotation2d()),
            new Pose2d(2, 2, new Rotation2d()),
            new Pose2d(3, 3, new Rotation2d(Units.degreesToRadians(160))),
        };
        public static final Integer patternMap = null;

        public static final double OUTER_ZONE = 2.262;
        public static final double INNER_ZONE = 1.131;
        public static final double RIN_STAR_BIN = 0.1;

        public static final int LPI_ROTATIONAL_DEADBAND = 1;

        public static final Pair<Integer, Integer> LEFT_WHEEL_LED_RANGE = new Pair<Integer, Integer>(
                0,
                10
        );

        public static final int NORTH = 0;
        public static final int NORTHEAST = 1;
        public static final int EAST = 2;
        public static final int SOUTHEAST = 3;
        public static final int SOUTHWEST = 4;
        public static final int SOUTH = 5;
        public static final int WEST = 6;
        public static final int NORTHWEST = 7;

        public static final HashMap<Integer, Pair<Integer, Integer>> ARROW_MAP = new HashMap<Integer, Pair<Integer, Integer>>() 
        {{
            put(NORTH, Pair.of(1, 2));
            put(NORTHEAST, Pair.of(3, 4));
            put(EAST, Pair.of(5, 6));
            put(SOUTHEAST, Pair.of(7, 8));
            put(SOUTHWEST, Pair.of(9, 10));
            put(SOUTH, Pair.of(11, 12));
            put(WEST, Pair.of(13, 14));
            put(NORTHWEST, Pair.of(15, 16));
        }};

    }

    public static final class NeoMotorConstants {
        public static final double VORTEX_FREE_SPEED_RPM = 6784;
        public static final double NEO_FREE_SPEED_RPM = 5676;

        public static final int MAX_PERIODIC_STATUS_TIME_MS = 65535;
        public static final int FAST_PERIODIC_STATUS_TIME_MS = 10;
      
        public static ArrayList<Neo> motors = new ArrayList<>();
    }

    public static final class IntakeConstants {
        public static final int INTAKE_CAN_ID = 9;
        public static final int TRIGGER_WHEEL_CAN_ID = 10;

        // % speeds of the motor
        public static final double INTAKE_PERCENT = 1;
        public static final double OUTTAKE_PERCENT = -1;
        public static final double STOP_PERCENT = 0;

        public static final int INTAKE_CURRENT_LIMIT_AMPS = 20;

        public static final double INTAKE_PERCENT_UPPER_LIMIT = 1;
        public static final double INTAKE_PERCENT_LOWER_LIMIT = -1;

        public static final double INDEXER_PERCENT_UPPER_LIMIT = 1;
        public static final double INDEXER_PERCENT_LOWER_LIMIT = -1;

        public static final int HAS_PIECE_CURRENT_THRESHOLD = 20;

        // TODO: Add these to the robot
        public static final int TRIGGER_WHEEL_CURRENT_LIMIT_AMPS = 30;
        public static final double SHOOTER_TRIGGER_WHEEL_PERCENT = -1;
        public static final double TRAP_TRIGGER_WHEEL_PERCENT = 1;
    }

    public static final class FieldConstants {

        public static boolean IS_SIMULATION = Robot.isSimulation();

        public static final double ALIGNMENT_SPEED = 3;
        public static final double SNAP_TO_ANGLE_P = 0.0025;

        public static final double ALLOWABLE_ERROR_METERS = Units.inchesToMeters(2);
        public static final double FIELD_WIDTH_METERS = 16.5410515;
        public static final double FIELD_HEIGHT_METERS = 8.2112312;
        public static final double CHAIN_HEIGHT_METERS = Units.feetToMeters(4);
        public static final double SPEAKER_HEIGHT_METERS = 2.082813;

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
        public static final Pose2d[] CHAIN_POSITIONS = new Pose2d[] {
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

        public static final Pose3d[] CHAIN_POSE3DS = new Pose3d[] {
            new Pose3d(CHAIN_POSITIONS[0]).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d())),
            new Pose3d(CHAIN_POSITIONS[1]).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d())),
            new Pose3d(CHAIN_POSITIONS[2]).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d())),
            new Pose3d(CHAIN_POSITIONS[3]).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d())),
            new Pose3d(CHAIN_POSITIONS[4]).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d())),
            new Pose3d(CHAIN_POSITIONS[5]).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d())),
        };

        // Speaker Positions: Blue alliance left
        // @formatter:off
        //
        //  0             1
        //
        //
        // @formatter:on
        private static final Pose2d[] SPEAKER_POSITIONS = new Pose2d[] {
                // All points are in meters and radians
                // All relative to the blue origin
                // Blue Speaker
                new Pose2d(0, 5.547, Rotation2d.fromDegrees(0)),
                // Red Speaker
                new Pose2d(FIELD_WIDTH_METERS, 5.547, Rotation2d.fromDegrees(0)),
        };

        public static final double SPEAKER_HEIGHT = 2.08;

        private static final Pose2d[] AMP_POSITIONS = new Pose2d[] {
                // All points are in meters and radians
                // All relative to the blue origin
                // Blue Amp
                new Pose2d(1.827, FIELD_HEIGHT_METERS, Rotation2d.fromDegrees(-90)),
                // Red Amp
                new Pose2d(14.706, FIELD_HEIGHT_METERS, Rotation2d.fromDegrees(-90)),
        };

        public static Pose2d GET_SPEAKER_POSITION() {
            return SPEAKER_POSITIONS[ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red) ? 1 : 0];
        } 

        public static Pose2d GET_AMP_POSITION() {
            return AMP_POSITIONS[ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red) ? 1 : 0];
        }

        // TODO: make real constants
        public static final Pose2d L_POSE = new Pose2d();
        public static final Pose2d R_POSE = new Pose2d();
        public static final Pose2d M_POSE = new Pose2d();

        public static final double CHAIN_LENGTH_METERS = Units.inchesToMeters(100);

        public static double CENTERLINE_X = FIELD_WIDTH_METERS / 2.0;

        // need to update
        private static double CENTERLINE_FIRST_Y = Units.inchesToMeters(29.638);
        private static double CENTERLINE_SEPARATION_Y = Units.inchesToMeters(66);
        private static double SPIKE_X = Units.inchesToMeters(114);
        // need
        private static double SPIKE_FIRST_Y = Units.inchesToMeters(161.638);
        private static double SPIKE_SEPARATION_Y = Units.inchesToMeters(57);
        private static double NOTE_Z = Units.inchesToMeters(2);

        private static Translation3d[] CENTERLINE_TRANSLATIONS = new Translation3d[5];
        private static Translation3d[] SPIKE_TRANSLATIONS_BLUE = new Translation3d[3];
        private static Translation3d[] SPIKE_TRANSLATIONS_RED = new Translation3d[3];
        public static Translation3d[] NOTE_TRANSLATIONS = new Translation3d[5 + 3 + 3]; // all staged + preload

        static {
            for (int i = 0; i < SPIKE_TRANSLATIONS_BLUE.length; i++) {
                SPIKE_TRANSLATIONS_BLUE[i] = new Translation3d(SPIKE_X, SPIKE_FIRST_Y + (i * SPIKE_SEPARATION_Y), NOTE_Z);
                SPIKE_TRANSLATIONS_RED[i] = new Translation3d(FIELD_WIDTH_METERS - SPIKE_X, SPIKE_FIRST_Y + (i * SPIKE_SEPARATION_Y), NOTE_Z);
            }
            for (int i = 0; i < CENTERLINE_TRANSLATIONS.length; i++) {
                CENTERLINE_TRANSLATIONS[i] =
                        new Translation3d(CENTERLINE_X, CENTERLINE_FIRST_Y + (i * CENTERLINE_SEPARATION_Y), NOTE_Z);
            }
            System.arraycopy(SPIKE_TRANSLATIONS_BLUE, 0, NOTE_TRANSLATIONS, 0, SPIKE_TRANSLATIONS_BLUE.length);
            System.arraycopy(SPIKE_TRANSLATIONS_RED, 0, NOTE_TRANSLATIONS, SPIKE_TRANSLATIONS_BLUE.length, SPIKE_TRANSLATIONS_RED.length);
            System.arraycopy(CENTERLINE_TRANSLATIONS, 0, NOTE_TRANSLATIONS, SPIKE_TRANSLATIONS_BLUE.length + SPIKE_TRANSLATIONS_RED.length, CENTERLINE_TRANSLATIONS.length);
        }
    }
    
    public static final class NTConstants {
        public static final int PIVOT_INDEX = 0;
        public static final int CLAW_INDEX = 1;
        public static final int ELEVATOR_INDEX = 2;
        public static final int LEFT_CLIMB_INDEX = 4;
        public static final int RIGHT_CLIMB_INDEX = 3;
        
        public static final double PIVOT_OFFSET_X = 0.112;
        public static final double PIVOT_OFFSET_Z = 0.21;

    
        public static final Translation3d PIVOT_OFFSET_METERS = new Translation3d(
            PIVOT_OFFSET_X,
            0, 
            PIVOT_OFFSET_Z);
        
    }

    public static final double GRAVITY = 9.8;

    public static final long LIMELIGHT_MAX_UPDATE_TIME = 200_000; // Micro Seconds = 0.2 Seconds

}