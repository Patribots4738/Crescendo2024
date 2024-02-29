package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.util.custom.PatrIDConstants;
import frc.robot.util.custom.SpeedAngleTriplet;
import frc.robot.util.rev.Neo;
import monologue.Logged;

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
        public static final double TRACK_WIDTH = Units.inchesToMeters(25.5);
        // Distance between front and back wheels on robot
        // Easiest measured from the center of the bore of the vortex
        public static final double WHEEL_BASE = Units.inchesToMeters(25.5);

        public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(29);
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

        public static final PatrIDConstants SHOOTER_PID = new PatrIDConstants(
            0.002,
            0,
            0.006,
            0.0001762
        );

        public static final PatrIDConstants PIVOT_PID = new PatrIDConstants(
            0.05,
            0,
            0.0083,
            -0.25,
            0.25
        );

        public static final int SHOOTER_CURRENT_LIMIT = 80;
        public static final int PIVOT_CURRENT_LIMIT = 15;

        public static final double SHOOTER_BACK_SPEED = -0.5;

        public static final double PIVOT_DEADBAND = 1;
        public static final double SHOOTER_RPM_DEADBAND = 50;

        public static final double PIVOT_LOWER_LIMIT_DEGREES = 16; 
        public static final double PIVOT_UPPER_LIMIT_DEGREES = 60;
        
        // This mega decimal number was gotten from REV hardware client, 
        // and we zeroed the pivot at its minimum angle
        public static final double ABSOLUTE_ENCODER_ZERO_OFFSET = 128.2069087-PIVOT_LOWER_LIMIT_DEGREES;


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
                // put(5, SpeedAngleTriplet.of(1930.0, 1930.0, 56.0));
                // put(6, SpeedAngleTriplet.of(2088.0, 2088.0, 50.0)); 
                // put(7, SpeedAngleTriplet.of(2188.0, 2188.0, 45.7));
                // put(8, SpeedAngleTriplet.of(2313.0, 2313.0, 41.3));
                // put(9, SpeedAngleTriplet.of(2465.0, 2465.0, 40.5));
                // put(10, SpeedAngleTriplet.of(2633.0, 2633.0, 38.1));
                // put(11, SpeedAngleTriplet.of(2795.0, 2795.0, 35.8));
                // put(12, SpeedAngleTriplet.of(2993.0, 2993.0, 34.3)); 
                // put(13, SpeedAngleTriplet.of(3526.0, 3526.0, 32.3));
                // put(14, SpeedAngleTriplet.of(3561.0, 3561.0, 31.0));
                put(15, SpeedAngleTriplet.of(3756.0, 3756.0, 29.9));
                put(16, SpeedAngleTriplet.of(3928.0, 3928.0, 30.0));
                put(17, SpeedAngleTriplet.of(3928.0, 3928.0, 29.1));

                // V2
                put(6, SpeedAngleTriplet.of(2127.0, 2127.0, 48.3));
                put(7, SpeedAngleTriplet.of(2127.0, 2127.0, 46.0));
                put(8, SpeedAngleTriplet.of(2528.0, 2528.0, 41.9));
                put(9, SpeedAngleTriplet.of(2600.0, 2600.0, 37.5));
                put(10, SpeedAngleTriplet.of(2884.0, 2894.0, 37.1));
                put(11, SpeedAngleTriplet.of(2924.0, 2930.0, 33.3));
                put(12, SpeedAngleTriplet.of(2924.0, 2930.0, 32.3));
                // V3 (note inside of indexer)
                put(13, SpeedAngleTriplet.of(3311.0, 3034.0, 30.3));
                put(14, SpeedAngleTriplet.of(3589.0, 3312.0, 30.3));
                // 2/28/24 - this was the day after bands were added
                put(15, SpeedAngleTriplet.of(3726.0, 3600.0, 27.3));

                // Random untested schenanagains
                put(18, SpeedAngleTriplet.of(3928.0, 3928.0, 28.6));
                put(19, SpeedAngleTriplet.of(4200.0, 4200.0, 28.3));
                put(20, SpeedAngleTriplet.of(4200.0, 4200.0, 27.9));

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
        public static final double GRAVITY = 9.8;
        public static final double V0Z = Math.sqrt(ShooterConstants.GRAVITY*2*FieldConstants.SPEAKER_HEIGHT_METERS);

    }

    public static final class TrapConstants {
        public static final int ELEVATOR_CAN_ID = 14;
        public static final int TRAP_CAN_ID = 15;
        public static final double ELEVATOR_DEADBAND = 0.03;
        public static final double GUILLOTINE_DEADBAND = 0.04;
        public static final double OUTTAKE_SECONDS = 3;
        public static final double TRAPPER_POSITION_MULTIPLIER = 1.925;

        public static final int TRAP_CURRENT_LIMIT = 15;
        public static final boolean TRAP_INVERTION = true;

        private static final double GEAR_RATIO = 16/Units.inchesToMeters(5.5);
        private static final double ELEVATOR_HEIGHT = Units.inchesToMeters(19)*2.0;

        public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.0 / 1.0/(GEAR_RATIO*ELEVATOR_HEIGHT);
        public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 40; // amps

        public static final PatrIDConstants ELEVATOR_PID = new PatrIDConstants(10, 0, 0);

        // TODO: set these values
        public static final double BOTTOM_POS = 0;
        public static final double INTAKE_TIME = 0;
        public static final double TRAPPER_OUTTAKE_PERCENT = -1;
        public static final double TRAPPER_INTAKE_PERCENT = 1;
        public static final double TRAPPER_OUTTAKE_SLOW = -0.3;
        public static final double TRAPPER_STOP_PERCENT = 0;
        public static final double TRAP_PLACE_POS = 0.49;
        public static final double AMP_PLACE_POS = 0.37;
        public static final double INDEX_POS = 0.09;
        public static final double DROP_POS = 0.11;
        public static final double GUILLOTONE_POS = 0.224;
        public static final double UNSTUCK_POS = 0.175;

        public static final double STUCK_TIME_SECONDS = 0.25;
        public static final double UNSTUCK_OUTTAKE_TIME_SECONDS = 0.3;

        public static final double ELEVATOR_TOP_LIMIT = 0.49;
        public static final double ELEVATOR_BOTTOM_LIMIT = 0;

        public static final double TRAPPER_LOWER_PERCENT_LIMIT = -1;
        public static final double TRAPPER_UPPER_PERCENT_LIMIT = 1;

        public static final double TRAPPER_HAS_PIECE_UPPER_LIMIT = 0;
        public static final double TRAPPER_HAS_PIECE_LOWER_LIMIT = -0.25;

        public static final double TRAPPER_HAS_PIECE_MIN_TIMESTAMP = 0.25;

        public static final double TRAPPER_HAS_PIECE_MIN_CURRENT = 15;

        public static final double TRAPPER_HAS_PIECE_MIN_TARGET_VELO = 0.45;
    }

    public static final class ClimbConstants {

        public static final int  LEFT_CLIMB_CAN_ID = 16;
        public static final int RIGHT_CLIMB_CAN_ID = 17;

        private static final double GEAR_RATIO = 16.0/Units.inchesToMeters(3.0);
        private static final double CLIMB_HEIGHT = Units.inchesToMeters(20)*2.0;
        
        public static final double CLIMB_POSITION_CONVERSION_FACTOR = 1.0/(GEAR_RATIO*CLIMB_HEIGHT);
        public static final int CLIMB_CURRENT_LIMIT = 40;

        public static final PatrIDConstants CLIMB_PID = new PatrIDConstants(5, 0, 0);

        public static final double EXTENSION_LIMIT_METERS = Units.feetToMeters(3.65);
        
        public static final double TOP_LIMIT = 0.517;
        public static final double BOTTOM_LIMIT = 0.0;

        public static final double CLIMB_DEADBAND = 0.002;
        
        public static final double DISTANCE_FROM_ORIGIN_METERS = 0.3048;
    }

    public static final class AutoConstants {

        // The below values need to be tuned for each new robot.
        // They are currently set to the values suggested by Choreo
        public static final double MAX_SPEED_METERS_PER_SECOND = 6.08;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 7.378;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI/4.0;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI/3.0;

        public static final double AUTO_POSITION_TOLERANCE_METERS = 0.2;
        public static final double AUTO_POSITION_TOLERANCE_RADIANS = 0.2;

        /*
         * XY:
         *  P: 5.2
         *  I: 0.125
         *  D: 0.0125
         * 
         * Theta:
         *   P: 1.3325
         *   I: 1 (izone on 20 degrees)
         *   D: 0.0375
         */
        public static final double XY_CORRECTION_P = 4;
        public static final double XY_CORRECTION_I = 0.0125;
        public static final double XY_CORRECTION_D = 0.0125;

        private static final PIDController XY_PID = new PIDController(
                AutoConstants.XY_CORRECTION_P,
                0,
                AutoConstants.XY_CORRECTION_D);

        public static final double ROTATION_CORRECTION_P = .3;
        public static final double ROTATION_CORRECTION_I = 0;
        public static final double ROTATION_CORRECTION_D = 0;

        public static final double PIECE_SEARCH_OFFSET_METERS = 1.0;

        public static final boolean USE_OBJECT_DETECTION = false;

        private static final ProfiledPIDController THETA_PID = new ProfiledPIDController(
            AutoConstants.ROTATION_CORRECTION_P,
            AutoConstants.ROTATION_CORRECTION_I,
            AutoConstants.ROTATION_CORRECTION_D,
            new TrapezoidProfile.Constraints(
                    AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                    AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)) 
            {{
                setIZone(Units.degreesToRadians(45));
            }};

        // Constraint for the motion-profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static HolonomicDriveController HDC = new HolonomicDriveController(
                XY_PID,
                XY_PID,
                THETA_PID
            );

        public static HolonomicPathFollowerConfig HPFC = new HolonomicPathFollowerConfig(
            new PIDConstants(
                AutoConstants.XY_CORRECTION_P,
                AutoConstants.XY_CORRECTION_I,
                AutoConstants.XY_CORRECTION_D),
            new PIDConstants(
                    AutoConstants.ROTATION_CORRECTION_P,
                    1,
                    AutoConstants.ROTATION_CORRECTION_D,
                    Units.degreesToRadians(45)),
            MAX_SPEED_METERS_PER_SECOND,
            Math.hypot(DriveConstants.WHEEL_BASE, DriveConstants.TRACK_WIDTH)/2.0,
            new ReplanningConfig());

        // In choreo, there is one path, "C1-5S", 
        // that shoots every piece.
        // There is a setting that splits the trajectories
        // by each stop point, and the auto generated
        // name by default is (PATH_NAME + "." + index)
        // so this represents that "." :>
        private static final String PATH_EXTENSION = ".";

        public static final String SHOOTING_DOWN_PATH_NAME = "C1-5S" + PATH_EXTENSION;
        public static final String SHOOTING_UP_PATH_NAME   = "C5-1S" + PATH_EXTENSION;

        public static final String SKIPPING_DOWN_PATH_NAME = "C1-5"  + PATH_EXTENSION;
        public static final String SKIPPING_UP_PATH_NAME   = "C5-1"  + PATH_EXTENSION;

        public static final String[] AUTO_NAMES = new String[] {
            "A W1 A C1-4 S",
            "S W1 A C1-5 S",
            "S W2 S C1-3 S W3-1 S",
            "S W2 S C1-4 S",
            "S C1-5 S",
            "S W1 A C1-5 S",
            "S W3-1 S",
            "S W3-1 S C1-3 S"
        };
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears:
        // L1 = 12T (slow, high torque)
        // L2 = 13T (medium)
        // L3 = 14T (fast, low torque)
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a robot that drives faster).
        // Extra high speed settings from REV make this number high thern L3
        public static final int DRIVING_MOTOR_PINION_TEETH = 16;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        // Update 2-29-24: I don't know how this needed to turn false but it is now false... -aleg
        public static final boolean TURNING_ENCODER_INVERTED = false;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.VORTEX_FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0.0762;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 19-22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double SPUR_GEAR_TEETH = 19;
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * SPUR_GEAR_TEETH) / (DRIVING_MOTOR_PINION_TEETH * 15);
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

        public static final double DRIVING_P = 0.256;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0.255;
        public static final double DRIVING_FF = 0.20217;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;
        public static final PatrIDConstants DRIVING_PID = new PatrIDConstants(
            DRIVING_P,
            DRIVING_I,
            DRIVING_D,
            DRIVING_FF,
            DRIVING_MIN_OUTPUT,
            DRIVING_MAX_OUTPUT
        );

        public static final double TURNING_P = Robot.isSimulation() ? 0.5 : 1.5;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 1;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;
        public static final PatrIDConstants TURNING_PID = new PatrIDConstants(
            TURNING_P,
            TURNING_I,
            TURNING_D,
            TURNING_FF,
            TURNING_MIN_OUTPUT,
            TURNING_MAX_OUTPUT
        );

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

        public static final double ALIGNMENT_DEADBAND = 0.2;

        // See https://www.desmos.com/calculator/e07raajzh5
        // And
        // https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;
    }

    public static final class LEDConstants {
        public static final int PWM_PORT = 9;
        public static final int LED_COUNT = new AddressableLEDBuffer(PWM_PORT).getLength();

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
        public static final boolean SAFE_SPARK_MODE = false;
        public static final double VORTEX_FREE_SPEED_RPM = 6784;
        public static final double NEO_FREE_SPEED_RPM = 5676;

        public static final int MAX_PERIODIC_STATUS_TIME_MS = 65535;
        public static final int FAST_PERIODIC_STATUS_TIME_MS = 20;
      
        // This gets filled out as motors are created on the robot
        public static final HashMap<Integer, Neo> MOTOR_MAP = new HashMap<Integer, Neo>();

        public static final HashMap<Integer, String> CAN_ID_MAP = new HashMap<Integer, String>() {{
            /*  1  */ put(DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID, "FrontRightDrive");
            /*  2  */ put(DriveConstants.FRONT_RIGHT_TURNING_CAN_ID, "FrontRightTurn");
            /*  3  */ put(DriveConstants.FRONT_LEFT_DRIVING_CAN_ID, "FrontLeftDrive");
            /*  4  */ put(DriveConstants.FRONT_LEFT_TURNING_CAN_ID, "FrontLeftTurn");
            /*  5  */ put(DriveConstants.REAR_LEFT_DRIVING_CAN_ID, "RearLeftDrive");
            /*  6  */ put(DriveConstants.REAR_LEFT_TURNING_CAN_ID, "RearLeftTurn");
            /*  7  */ put(DriveConstants.REAR_RIGHT_DRIVING_CAN_ID, "RearRightDrive");
            /*  8  */ put(DriveConstants.REAR_RIGHT_TURNING_CAN_ID, "RearRightTurn");
            /*  9  */ put(IntakeConstants.INTAKE_CAN_ID, "Intake");
            /* 10  */ put(IntakeConstants.TRIGGER_WHEEL_CAN_ID, "TriggerWheel");
            /* 11  */ put(ShooterConstants.LEFT_SHOOTER_CAN_ID, "LeftShooter");
            /* 12  */ put(ShooterConstants.RIGHT_SHOOTER_CAN_ID, "RightShooter");
            /* 13  */ put(ShooterConstants.SHOOTER_PIVOT_CAN_ID, "ShooterPivot");
            /* 14  */ put(TrapConstants.ELEVATOR_CAN_ID, "Elevator");
            /* 15  */ put(TrapConstants.TRAP_CAN_ID, "Trap");
            /* 16  */ put(ClimbConstants.LEFT_CLIMB_CAN_ID, "LeftClimb");
            /* 17  */ put(ClimbConstants.RIGHT_CLIMB_CAN_ID, "RightClimb");
        }};

        public static final HashMap<String, List<Neo>> MOTOR_GROUPS = new HashMap<String, List<Neo>>();

        public static Map<String, List<Neo>> initializeMotorGroupMap() {
            MOTOR_GROUPS.put("Drive", new ArrayList<Neo>() {{
                add(MOTOR_MAP.get(DriveConstants.FRONT_LEFT_DRIVING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.REAR_LEFT_DRIVING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.REAR_RIGHT_DRIVING_CAN_ID));
            }});
            MOTOR_GROUPS.put("Turn", new ArrayList<Neo>() {{
                add(MOTOR_MAP.get(DriveConstants.FRONT_LEFT_TURNING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.FRONT_RIGHT_TURNING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.REAR_LEFT_TURNING_CAN_ID));
                add(MOTOR_MAP.get(DriveConstants.REAR_RIGHT_TURNING_CAN_ID));
            }});
            MOTOR_GROUPS.put("Shooter", new ArrayList<Neo>() {{
                add(MOTOR_MAP.get(ShooterConstants.LEFT_SHOOTER_CAN_ID));
                add(MOTOR_MAP.get(ShooterConstants.RIGHT_SHOOTER_CAN_ID));
            }});
            MOTOR_GROUPS.put("Pivot", new ArrayList<Neo>() {{
                add(MOTOR_MAP.get(ShooterConstants.SHOOTER_PIVOT_CAN_ID));
            }});
            MOTOR_GROUPS.put("Elevator", new ArrayList<Neo>() {{
                add(MOTOR_MAP.get(TrapConstants.ELEVATOR_CAN_ID));
            }});
            MOTOR_GROUPS.put("Climb", new ArrayList<Neo>() {{
                add(MOTOR_MAP.get(ClimbConstants.LEFT_CLIMB_CAN_ID));
                add(MOTOR_MAP.get(ClimbConstants.RIGHT_CLIMB_CAN_ID));
            }});

            return MOTOR_GROUPS;
        }
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

        public static final int TRIGGER_WHEEL_CURRENT_LIMIT_AMPS = 30;
        public static final double SHOOTER_TRIGGER_WHEEL_PERCENT = -1;
        public static final double TRAP_TRIGGER_WHEEL_PERCENT = 1;
    }

    public static final class FieldConstants {

        public static boolean IS_SIMULATION = Robot.isSimulation();
        public static final int CENTER_NOTE_COUNT = 5;

        public static final Pose2d BLUE_ORIGIN = new Pose2d(0, 0, new Rotation2d());

        public static final double ALIGNMENT_SPEED = 3;
        public static final double SNAP_TO_ANGLE_P = 0.0025;

        public static final double ALLOWABLE_ERROR_METERS = Units.inchesToMeters(2);
        public static final double FIELD_WIDTH_METERS = 16.5410515;
        public static final double FIELD_HEIGHT_METERS = 8.2112312;
        public static final double CHAIN_HEIGHT_METERS = Units.feetToMeters(4);
        public static final double SPEAKER_HEIGHT_METERS = 2.082813;

        // Field:
        // https://cad.onshape.com/documents/dcbe49ce579f6342435bc298/w/b93673f5b2ec9c9bdcfec487/e/6ecb2d6b7590f4d1c820d5e3
        // Chain Positions: Blue alliance left
        // @formatter:off
        //  2              5
        //      1      4
        //  0              3
        // @formatter:on
        public static final List<Pose2d> CHAIN_POSITIONS = new ArrayList<Pose2d>() {{
            // All points are in meters and radians
            // All relative to the blue origin
            // Blue Stage
            Pose2d bluePose1 = new Pose2d(4.37, 3.201, Rotation2d.fromDegrees(-120));
            Pose2d bluePose2 = new Pose2d(5.875, 4.168, Rotation2d.fromDegrees(0));
            Pose2d bluePose3 = new Pose2d(4.353, 4.938, Rotation2d.fromDegrees(120));
            add(bluePose1);
            add(bluePose2);
            add(bluePose3);

            // Red Stage
            add(GeometryUtil.flipFieldPose(bluePose1));
            add(GeometryUtil.flipFieldPose(bluePose2));
            add(GeometryUtil.flipFieldPose(bluePose3));
        }};

        public static final List<Pose2d> STAGE_POSITIONS = new ArrayList<Pose2d>() {{
            // All points are in meters and radians
            // All relative to the blue origin
            // Blue Stage
            Pose2d blueStage = new Pose2d(4.897, 4.064, new Rotation2d());
            add(blueStage);

            // Red Stage
            add(GeometryUtil.flipFieldPose(blueStage));
        }};

        public static final List<Pose2d> SOURCE_POSITIONS = new ArrayList<Pose2d>() {{
            // All points are in meters and radians
            // All relative to the blue origin
            // Blue Source
            Pose2d blueSource = new Pose2d(15.452, 0.971, Rotation2d.fromDegrees(120));
            add(blueSource);

            // Red Source
            add(GeometryUtil.flipFieldPose(blueSource));
        }};

        public static final List<Pose3d> CHAIN_POSE3DS = new ArrayList<Pose3d>() {{
            // All points are in meters and radians
            // All relative to the blue origin
            // Blue Chain
            Pose3d blueChain1 = new Pose3d(CHAIN_POSITIONS.get(0)).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d()));
            Pose3d blueChain2 = new Pose3d(CHAIN_POSITIONS.get(1)).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d()));
            Pose3d blueChain3 = new Pose3d(CHAIN_POSITIONS.get(2)).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d()));
            add(blueChain1);
            add(blueChain2);
            add(blueChain3);

            // Red Chain
            add(new Pose3d(GeometryUtil.flipFieldPose(blueChain1.toPose2d())).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d())));
            add(new Pose3d(GeometryUtil.flipFieldPose(blueChain2.toPose2d())).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d())));
            add(new Pose3d(GeometryUtil.flipFieldPose(blueChain3.toPose2d())).plus(new Transform3d(0.0, 0.0, CHAIN_HEIGHT_METERS, new Rotation3d())));
        }};

        // Speaker Positions: Blue alliance left
        // @formatter:off
        //
        //  0             1
        //
        //
        // @formatter:on
        public static final List<Pose2d> SPEAKER_POSITIONS = new ArrayList<Pose2d>() {{
            // All points are in meters and radians
            // All relative to the blue origin
            // Blue Speaker
            Pose2d blueSpeaker = new Pose2d(0, 5.547, Rotation2d.fromDegrees(0));
            Pose2d redSpeaker = GeometryUtil.flipFieldPose(blueSpeaker).plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180)));
            add(blueSpeaker);
            add(redSpeaker);
        }};
    
        public static final List<Pose2d> AMP_POSITIONS = new ArrayList<Pose2d>() {{
            // All points are in meters and radians
            // All relative to the blue origin
            Pose2d bluePose = new Pose2d(1.827, FIELD_HEIGHT_METERS, Rotation2d.fromDegrees(-90));
            Pose2d redPose = GeometryUtil.flipFieldPose(bluePose);
            add(bluePose);
            add(redPose);
        }};

        private static int getAllianceIndex(Alliance defaultAlliance) {
            return defaultAlliance == Alliance.Blue
                ? (Robot.isRedAlliance() ? 1 : 0) 
                : (Robot.isBlueAlliance() ? 1 : 0);
        }

        public static Pose2d GET_SPEAKER_POSITION() {
            return SPEAKER_POSITIONS.get(getAllianceIndex(Alliance.Blue));
        }

        public static Pose2d GET_SOURCE_POSITION() {
            return SOURCE_POSITIONS.get(getAllianceIndex(Alliance.Blue));
        }

        public static Translation2d GET_SPEAKER_TRANSLATION() {
            return GET_SPEAKER_POSITION().getTranslation();
        }

        public static Pose2d GET_AMP_POSITION() {
            return AMP_POSITIONS.get(getAllianceIndex(Alliance.Blue));
        }

        public static Pose2d GET_STAGE_POSITION() {
            return STAGE_POSITIONS.get(getAllianceIndex(Alliance.Blue));
        }

        public static List<Pose2d> GET_CHAIN_POSITIONS() {
            int startIndex = Robot.isRedAlliance() ? 3 : 0;
            return CHAIN_POSITIONS.subList(startIndex, startIndex + 3);
        }

        public static final double CHAIN_LENGTH_METERS = Units.inchesToMeters(100);

        public static double CENTERLINE_X = FIELD_WIDTH_METERS / 2.0;
        public static double BLUE_WING_X = 5.84;
        public static double RED_WING_X = 10.7;

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
        public static Translation3d[] HIGH_NOTE_TRANSLATIONS = new Translation3d[5 + 3 + 3]; // all staged + preload

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
            for (int i = 0; i < NOTE_TRANSLATIONS.length; i++) {
                Translation3d noteTranslation3d = NOTE_TRANSLATIONS[i];
                HIGH_NOTE_TRANSLATIONS[i] = new Translation3d(noteTranslation3d.getX(), noteTranslation3d.getY(), noteTranslation3d.getZ() - 0.1);
            }
        }

        public static final Translation2d L_POSE = new Translation2d(5.11,6.23);
        public static final Translation2d R_POSE = new Translation2d(5.88, 1.84);
        public static final Translation2d M_POSE = new Translation2d(4.46,4.81);
        public static final Translation2d W3_POSE = SPIKE_TRANSLATIONS_BLUE[0].toTranslation2d();

        public static final List<Pose2d> SHOOTING_POSITIONS = new ArrayList<Pose2d>() {{
            Pose2d L_POSE2D = new Pose2d(L_POSE, Rotation2d.fromDegrees(179.61));
            Pose2d R_POSE2D = new Pose2d(R_POSE, Rotation2d.fromDegrees(148.86));
            Pose2d M_POSE2D = new Pose2d(M_POSE, Rotation2d.fromDegrees(151.68));
            Pose2d W3_POSE2D = new Pose2d(W3_POSE, new Rotation2d());
            //Blue
            add(L_POSE2D);
            add(R_POSE2D);
            add(M_POSE2D);
            add(W3_POSE2D);
            //Red
            add(GeometryUtil.flipFieldPose(L_POSE2D));
            add(GeometryUtil.flipFieldPose(R_POSE2D));
            add(GeometryUtil.flipFieldPose(M_POSE2D));
            add(GeometryUtil.flipFieldPose(W3_POSE2D));
        }};

        public static List<Pose2d> GET_SHOOTING_POSITIONS() {
            int startingIndex = Robot.isRedAlliance() ? SHOOTING_POSITIONS.size() / 2 : 0;
            return SHOOTING_POSITIONS.subList(startingIndex, startingIndex + 4);
        }

        public static List<Pose2d> GET_CENTERLINE_NOTES() {
            List<Pose2d> notePoses = new ArrayList<Pose2d>();
            for (int i = CENTERLINE_TRANSLATIONS.length - 1; i >= 0; i--) {
                notePoses.add(
                    new Pose2d(
                        CENTERLINE_TRANSLATIONS[i].toTranslation2d(), 
                        new Rotation2d()));
            }
            return notePoses;
        }

        
    }

    public static final class CameraConstants {
        public static final long LIMELIGHT_MAX_UPDATE_TIME = 200_000; // Micro Seconds = 0.2 Seconds

        private static final double CAM_HEIGHT = Units.inchesToMeters(16);
        private static final double CAM_X = Units.inchesToMeters(6.6 / 2.0);
        private static final double CAM_Y = Units.inchesToMeters(15.3 / 2.0);
        private static final double CAM_PITCH = Units.degreesToRadians(-15);
        private static final double CAM_YAW = Units.degreesToRadians(32);

        private static final Pose3d cam1 =
            new Pose3d(
                new Translation3d(CAM_X, CAM_Y, CAM_HEIGHT), 
                new Rotation3d(0, CAM_PITCH, CAM_YAW));
        private static final Pose3d cam2 =
            new Pose3d(
                new Translation3d(CAM_X, -CAM_Y, CAM_HEIGHT),
                new Rotation3d(0, CAM_PITCH, -CAM_YAW));
        private static final Pose3d cam3 =
            new Pose3d(
                new Translation3d(-CAM_X, CAM_Y, CAM_HEIGHT),
                new Rotation3d(0, CAM_PITCH, (Math.PI) - CAM_YAW));
        private static final Pose3d cam4 =
            new Pose3d(
                new Translation3d(-CAM_X, -CAM_Y, CAM_HEIGHT),
                new Rotation3d(0, CAM_PITCH, (Math.PI) + CAM_YAW));

        private static final Pose3d cam5 = 
            new Pose3d(
                0.0508, -0.1524, 0.589701,
                new Rotation3d(0, Units.degreesToRadians(-33), Units.degreesToRadians(15)));
        private static final Pose3d cam6 =
        new Pose3d(
                -0.254, -0.155575, 0.589701,
                new Rotation3d(0, Units.degreesToRadians(180), Units.degreesToRadians(-10)));

        public static final Pose3d[] cameras = new Pose3d[] {cam5, cam6};
    }
    
    public static final class NTConstants {
        public static final int PIVOT_INDEX = 0;
        public static final int TRAPPER_INDEX = 1;
        public static final int ELEVATOR_INDEX = 2;
        public static final int RIGHT_CLIMB_INDEX = 3;
        public static final int LEFT_CLIMB_INDEX = 4;
        
        public static final double PIVOT_OFFSET_X = 0.165;
        public static final double PIVOT_OFFSET_Z = 0.2095;
    
        public static final Translation3d PIVOT_OFFSET_METERS = new Translation3d(
            PIVOT_OFFSET_X,
            0, 
            PIVOT_OFFSET_Z);
        
        public static final Map<String, Number> WAIT_TIMES = new HashMap<>() {{
            put("noteToShoot1", 0.7);
            put("noteToShoot2", 0.4);
            put("noteToShoot3", 0.0);

            put("intakeToTrap1", 0.5);
            put("intakeToTrap2", 0.0);

            put("noteToIndexer1", 0.6);
            put("noteToIndexer2", 0.07);
            put("noteToIndexer3", 0.0);

            put("noteToTrap1", 0.2);
            put("noteToTrap2", 0.5);
            put("noteToTrap3", 0.0);

            put("ampPosition", 0.37);
            put("atan++", 0.0);
        }};
    }
}
