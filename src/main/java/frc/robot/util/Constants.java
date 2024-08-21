package frc.robot.util;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.ColorMatch;
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
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
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

    public static final class StateConstants {

        private static RobotType robotType = RobotType.COMPBOT;

        public static RobotType getRobot() {
            if (!FieldConstants.IS_SIMULATION && robotType == RobotType.SIMBOT) {
                robotType = RobotType.COMPBOT;
            }
            return robotType;
        }

        public static Mode getMode() {
            return switch (robotType) {
                case COMPBOT -> FieldConstants.IS_SIMULATION ? Mode.REPLAY : Mode.REAL;
                case SIMBOT -> Mode.SIM;
            };
        }

        public static boolean isSimulation() {
            return getMode() == Mode.SIM;
        }
        
        public enum Mode {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }

        public enum RobotType {
            COMPBOT,
            SIMBOT
        }

    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static double MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;

        public static final double MAX_ANGULAR_SPEED_RADS_PER_SECOND = Units.degreesToRadians(1137.21); // radians per second

        public static final double MAX_TELEOP_SPEED_METERS_PER_SECOND = 4.7;

        public static final double PASS_ROTATION_DEADBAND = 10;

        public static final double ODOMETRY_FREQUENCY = 250.0;

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
        public static final double PIVOT_ZERO_OFFSET = 51.2235296;

        public static final PatrIDConstants SHOOTER_PID = new PatrIDConstants(
            0.002,
            0,
            0.006,
            0.0001762
        );

        public static final PatrIDConstants PIVOT_PID = new PatrIDConstants(
            0.07,
            0,
            0.0083
        );

        public static final int SHOOTER_CURRENT_LIMIT = 60;
        public static final int PIVOT_CURRENT_LIMIT = 20;

        public static final double SHOOTER_BACK_SPEED = -0.5;

        public static final double PIVOT_DEADBAND_DEGREES = 1;
        public static final double PIVOT_PASS_DEADBAND_DEGREES = 4;
        public static final double SHOOTER_RPM_DEADBAND = 50;
        public static final double SHOOTER_PASS_RPM_DEADBAND = 150;

        public static final double PIVOT_LOWER_LIMIT_DEGREES = 21;
        public static final double PIVOT_UPPER_LIMIT_DEGREES = 59;
        public static final double PIVOT_RAISE_ANGLE_DEGREES = 49;

        public static final SpeedAngleTriplet SHOOTER_AMP_TRIPLET = SpeedAngleTriplet.of(712.0, 554.0, 55.4);

        public static final double SHOOTER_RPM_LOWER_LIMIT = -NeoMotorConstants.NEO_FREE_SPEED_RPM;
        public static final double SHOOTER_RPM_UPPER_LIMIT = NeoMotorConstants.NEO_FREE_SPEED_RPM;

        public static final double DEFAULT_RPM = 2500;
        
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
                put(4, SpeedAngleTriplet.of(1763.0, 2316.0, 56.1));
                put(5, SpeedAngleTriplet.of(1763.0, 2316.0, 57.6));
                // put(6, SpeedAngleTriplet.of(2088.0, 2088.0, 50.0)); 
                // put(7, SpeedAngleTriplet.of(2188.0, 2188.0, 45.7));
                // put(8, SpeedAngleTriplet.of(2313.0, 2313.0, 41.3));
                // put(9, SpeedAngleTriplet.of(2465.0, 2465.0, 40.5));
                // put(10, SpeedAngleTriplet.of(2633.0, 2633.0, 38.1));
                // put(11, SpeedAngleTriplet.of(2795.0, 2795.0, 35.8));
                // put(12, SpeedAngleTriplet.of(2993.0, 2993.0, 34.3)); 
                // put(13, SpeedAngleTriplet.of(3526.0, 3526.0, 32.3));
                // put(14, SpeedAngleTriplet.of(3561.0, 3561.0, 31.0));
                // put(15, SpeedAngleTriplet.of(3756.0, 3756.0, 29.9));
                // put(16, SpeedAngleTriplet.of(3928.0, 3928.0, 30.0));
                // put(17, SpeedAngleTriplet.of(3928.0, 3928.0, 29.1));

                // V2
                // put(6, SpeedAngleTriplet.of(2127.0, 2127.0, 48.3));
                // put(7, SpeedAngleTriplet.of(2127.0, 2127.0, 46.0));
                // put(8, SpeedAngleTriplet.of(2528.0, 2528.0, 41.9));
                // put(9, SpeedAngleTriplet.of(2600.0, 2600.0, 37.5));
                // put(10, SpeedAngleTriplet.of(2884.0, 2894.0, 37.1));
                // put(11, SpeedAngleTriplet.of(2924.0, 2930.0, 33.3));
                // put(12, SpeedAngleTriplet.of(2924.0, 2930.0, 32.3));
                // // V3 (note inside of indexer)
                // put(13, SpeedAngleTriplet.of(3311.0, 3034.0, 30.3));
                // put(14, SpeedAngleTriplet.of(3589.0, 3312.0, 30.3));
                // // 2/28/24 - this was the day after bands were added
                // put(15, SpeedAngleTriplet.of(3726.0, 3600.0, 27.3));

                // put(16, SpeedAngleTriplet.of(3986.0, 3990.0, 33.7));
                // put(17, SpeedAngleTriplet.of(3986.0, 3990.0, 33));
                // put(18, SpeedAngleTriplet.of(3986.0, 3990.0, 32.5));
                // put(19, SpeedAngleTriplet.of(4201.0, 4205.0, 32.0));

                // put(7, SpeedAngleTriplet.of(2840.0, 2850.0, 44.6));
                // put(8, SpeedAngleTriplet.of(2810.0, 2820.0, 39.5));
                // put(9, SpeedAngleTriplet.of(2886.0, 2886.0, 38.4));
                // put(10, SpeedAngleTriplet.of(2888.0, 2897.0, 36.6));
                // // Driverstation has some words for you:
                // //   樀愀瘀愀㨀㐀㄀㈀⤀㨀 䰀漀漀瀀 琀椀洀攀 漀昀 　⸀　㈀猀 漀瘀攀爀爀甀渀 ＀෾਀＀￾￾￾￾￾⃾ ＀෾਀＀
                // put(11, SpeedAngleTriplet.of(2943.0, 2930.0, 36));
                // put(12, SpeedAngleTriplet.of(2943.0, 2935.0, 35.9));
                // put(13, SpeedAngleTriplet.of(3319.0, 3042.0, 35.3));
                // put(14, SpeedAngleTriplet.of(3587.0, 3310.0, 34.7));
                // put(15, SpeedAngleTriplet.of(3586.0, 3371.0, 34));

                // NEW SHOOTER WOOOHOO
                put(6, SpeedAngleTriplet.of(3007.0, 2850.0, 50.1));
                put(7, SpeedAngleTriplet.of(3160.0, 2865.0, 46.3));
                put(8, SpeedAngleTriplet.of(3310.0, 3017.0, 43.4));
                put(9, SpeedAngleTriplet.of(3502.0, 3202.0, 40.2));
                put(10, SpeedAngleTriplet.of(3706.0, 3305.0, 37.8));
                put(11, SpeedAngleTriplet.of(3856.0, 3539.0, 34.5));
                put(12, SpeedAngleTriplet.of(3947.0, 3580.0, 33.8));
                put(13, SpeedAngleTriplet.of(4075.0, 3691.0, 31.8));
                // Future note, 13.2ft is a common shot which should have its own calibration point
                put(14, SpeedAngleTriplet.of(4214.0, 3755.0, 30.9)); 
                put(15, SpeedAngleTriplet.of(4515.0, 4043.0, 30.9));
                put(16, SpeedAngleTriplet.of(4190.0, 3731.0, 26.7));
                
            }
        };

        public static final InterpolatingTreeMap<Double, SpeedAngleTriplet> INTERPOLATION_MAP = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(),
                SpeedAngleTriplet.getInterpolator()) {{
            for (Map.Entry<Integer, SpeedAngleTriplet> entry : SPEAKER_DISTANCES_TO_SPEEDS_AND_ANGLE_MAP.entrySet()) {
                put(entry.getKey().doubleValue(), entry.getValue());
            }
        }};

        public static final double SHOOTER_WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
        public static final double GRAVITY = 9.81;
        public static final double SPEAKER_V0Z = Math.sqrt(ShooterConstants.GRAVITY*2*FieldConstants.SPEAKER_HEIGHT_METERS);
        public static final double PASS_V0Z = Math.sqrt(ShooterConstants.GRAVITY*2*FieldConstants.PASS_HEIGHT_METERS);

    }

    public static final class ElevatorConstants {
        public static final int ELEVATOR_CAN_ID = 14;
        public static final int AMPPER_CAN_ID = 15;
        public static final double ELEVATOR_DEADBAND = 0.03;
        public static final double OUTTAKE_SECONDS = 3;
        public static final double AMPPER_POSITION_MULTIPLIER = 1.925;

        public static final int AMPPER_CURRENT_LIMIT = 15;
        public static final boolean AMPPER_INVERTION = true;

        private static final double GEAR_RATIO = 16/Units.inchesToMeters(5.5);
        private static final double ELEVATOR_HEIGHT = Units.inchesToMeters(19)*2.0;

        public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.0 / 1.0/(GEAR_RATIO*ELEVATOR_HEIGHT);
        public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 40; // amps

        public static final PatrIDConstants ELEVATOR_PID = new PatrIDConstants(10, 0, 0);

        // TODO: set these values
        public static final double BOTTOM_POS = -.005;
        public static final double INTAKE_TIME = 0;
        public static final double AMPPER_OUTTAKE_PERCENT = -1;
        public static final double AMPPER_INTAKE_PERCENT = 1;
        public static final double AMPPER_OUTTAKE_SLOW = -0.3;
        public static final double AMPPER_STOP_PERCENT = 0;
        public static final double TRAP_PLACE_POS = 0.49;
        public static final double DEBUG_ELEVATOR_POS = 0.47;
        // This position is not used
        public static final double AMP_PLACE_POS = 0.409;        
        public static final double NOTE_FIX_POS = 0.036;
        public static final double INDEX_POS = 0.09;
        public static final double DROP_POS = 0.11;
        public static final double GUILLOTONE_POS = 0.224;
        public static final double UNSTUCK_POS = 0.175;

        public static double PREP_PIECE_SECONDS = 0.3;

        public static final double STUCK_TIME_SECONDS = 0.25;
        public static final double UNSTUCK_OUTTAKE_TIME_SECONDS = 0.3;

        public static final double ELEVATOR_TOP_LIMIT = 0.49;
        public static final double ELEVATOR_BOTTOM_LIMIT = -0.0254;

        public static final double AMPPER_LOWER_PERCENT_LIMIT = -1;
        public static final double AMPPER_UPPER_PERCENT_LIMIT = 1;

        public static final double AMPPER_HAS_PIECE_UPPER_LIMIT = 0;
        public static final double AMPPER_HAS_PIECE_LOWER_LIMIT = -0.25;

        public static final double AMPPER_HAS_PIECE_MIN_TIMESTAMP = 0.25;

        public static final double AMPPER_HAS_PIECE_MIN_CURRENT = 15;

        public static final double AMPPER_HAS_PIECE_MIN_TARGET_VELO = 0.45;
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

        public static final double CLIMB_DEADBAND = 0.005;
        
        public static final double DISTANCE_FROM_ORIGIN_METERS = 0.3048;
    }

    public static final class AutoConstants {

        // The below values need to be tuned for each new robot.
        // They are currently set to the values suggested by Choreo
        public static final double MAX_SPEED_METERS_PER_SECOND = 5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5;
        // Below is gotten from choreo
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Units.degreesToRadians(1137.21);
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Units.degreesToRadians(792.90);

        public static final double AUTO_POSITION_TOLERANCE_METERS = Units.inchesToMeters(1);
        public static final double AUTO_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(2);
        public static final double PASS_ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(10);

        public static final double AUTO_ALIGNMENT_DEADBAND = Units.inchesToMeters(3);

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

        public static final double ROTATION_CORRECTION_P = 3.725;
        public static final double ROTATION_CORRECTION_I = 0;
        public static final double ROTATION_CORRECTION_D = 0;

        public static final double PIECE_SEARCH_OFFSET_METERS = 1.7;

        public static final boolean USE_OBJECT_DETECTION = true;

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
                    AutoConstants.ROTATION_CORRECTION_I,
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
            // "S C5-4 S OBJ",
            "8044 W3 C3",
            "S C5-1 S",
            "S C5-3 3-5 S",
            // "S W3-1 S",
            // "S W3-1 S 2",
            "S W3-1 S C2-5 S",
            "S W3-1 S C2-5 S 2",
            
            "S W3-1 S C1-2 S",
            "S W3-1 S C1-2 S 2",

            "S W3-1 S C3-5 S",
            "S W3-1 S",
            // "S W3-1 S C2-3 S 2 OBJ",
            // "S W3-1 S C2-3 S OBJ",
            // "S W3-1 C1-3 OBJ",
            // "S C1-3 OBJ Over W1",
            // "S C1-5 C5-1 OBJ Over W1",
            "S C1-5 S Over W1",
            // "S C5-1 C1-5 OBJ",
            // "Proxy"
        };
    }

    public static final class ColorSensorConstants {

        public static final I2C.Port I2C_PORT = I2C.Port.kMXP;
        public static final Color NOTE_ORANGE = new Color(0.957, 0.282, 0.086);
        public static final ColorMatch COLOR_MATCH = new ColorMatch() 
        {{
            // BLACK
            addColorMatch(Color.kBlack);
            // NEW COMP READY NOTE ORANGE
            addColorMatch(NOTE_ORANGE);
            addColorMatch(Color.kWhite);
        }};

    }
    public static final class ModuleConstants {

        // https://www.revrobotics.com/rev-21-3005/
        private enum SwerveGearing {
            LOW         (12, 22, 4.12, 4.92),
            MEDIUM      (13, 22, 4.46, 5.33),
            HIGH        (14, 22, 4.8, 5.5/*5.74*/),

            EXTRA_HIGH_1(14, 21, 5.03, 6.01),
            EXTRA_HIGH_2(14, 20, 5.28, 6.32),
            EXTRA_HIGH_3(15, 20, 5.66, 6.77),
            EXTRA_HIGH_4(16, 20, 6.04, 7.22),
            EXTRA_HIGH_5(16, 19, 6.36, 7.60);

            private final double 
                pinionTeeth, 
                spurTeeth, 
                maxSpeedNeo,
                maxSpeedVortex;

            SwerveGearing(
                int pinionTeeth, 
                int spurTeeth, 
                double maxSpeedNeo,
                double maxSpeedVortex)
            {
                this.pinionTeeth = pinionTeeth;
                this.spurTeeth = spurTeeth;
                this.maxSpeedNeo = maxSpeedNeo;
                this.maxSpeedVortex = maxSpeedVortex;
            }
        }

        public static final SwerveGearing CURRENT_GEARING = SwerveGearing.HIGH;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.VORTEX_FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.4989692140582187*2.0);
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 15 teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * CURRENT_GEARING.spurTeeth) / (CURRENT_GEARING.pinionTeeth * 15.0);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
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

        // See https://www.desmos.com/calculator/e07raajzh5
        // And
        // https://docs.google.com/spreadsheets/d/1Lytrh6q9jkz4u1gmF1Sk8kTpj8DxW-uwRE_QMnTt8Lk
        public static final double CONTROLLER_CORNER_SLOPE_1 = 1 / 0.7;
        public static final double CONTROLLER_CORNER_SLOPE_2 = 0.7;

        public static final boolean SINGLE_DRIVER_MODE = true;
    }

    public static final class LEDConstants {
        public static final int PWM_PORT = 9;
        public static final int LED_COUNT = new AddressableLEDBuffer(PWM_PORT).getLength();

        public static final double OUTER_ZONE = 2.262;
        public static final double INNER_ZONE = 1.131;
        public static final double RIN_STAR_BIN = Units.inchesToMeters(2.54);

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

        public static final int MAX_PERIODIC_STATUS_TIME_MS = 32766;
        public static final int FAST_PERIODIC_STATUS_TIME_MS = 15;
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
            /* 14  */ put(ElevatorConstants.ELEVATOR_CAN_ID, "Elevator");
            /* 15  */ put(ElevatorConstants.AMPPER_CAN_ID, "Amp");
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
                add(MOTOR_MAP.get(ElevatorConstants.ELEVATOR_CAN_ID));
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
        public static final double AMP_TRIGGER_WHEEL_PERCENT = 1;
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
        public static final double PASS_HEIGHT_METERS = Units.feetToMeters(13.5);

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

        public static final List<Pose2d> SUBWOOFER_POSITIONS = new ArrayList<Pose2d>() {{
            // All points are in meters and radians
            // All relative to the blue origin
            // Blue Source
            Pose2d blueSubwoofer = new Pose2d(1.29, 5.55, Rotation2d.fromDegrees(180));
            add(blueSubwoofer);

            // Red Source
            add(GeometryUtil.flipFieldPose(blueSubwoofer));
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
        
        public static final List<Pose2d> PASS_APEX_POSITIONS = new ArrayList<Pose2d>() {{

            // I swear bulldogs and hawaiin kids just had to get in here somehow
            Pose2d bluePose = new Pose2d(4.581, 4.359, Rotation2d.fromDegrees(0));
            Pose2d redPose = GeometryUtil.flipFieldPose(bluePose).plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180)));

            add(bluePose);
            add(redPose);
        }};

        public static final List<Pose2d> PASS_TARGET_POSITIONS = new ArrayList<Pose2d>() {{

            // I swear bulldogs and hawaiin kids just had to get in here somehow
            Pose2d bluePose = new Pose2d(1.07, 6.99, Rotation2d.fromDegrees(0));
            Pose2d redPose = GeometryUtil.flipFieldPose(bluePose).plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180)));

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

        public static Pose2d GET_SUBWOOFER_POSITION() {
            return SUBWOOFER_POSITIONS.get(getAllianceIndex(Alliance.Blue));
        }

        public static Pose2d GET_PASS_APEX_POSITION() {
            return PASS_APEX_POSITIONS.get(getAllianceIndex(Alliance.Blue));
        }

        public static Pose2d GET_PASS_TARGET_POSITION() {
            return PASS_TARGET_POSITIONS.get(getAllianceIndex(Alliance.Blue));
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

        public static final Translation2d L_POSE = new Translation2d(3.47, 5.89);
        public static final Translation2d R_POSE = new Translation2d(3, 3);
        public static final Translation2d M_POSE = new Translation2d(3.95,5.060);
        public static final Translation2d W3_POSE = SPIKE_TRANSLATIONS_BLUE[0].toTranslation2d();

        public static final List<Translation2d> TAG_CALIBRATION_POSE = new ArrayList<>(){{
            Translation2d W1_BLUE = new Translation2d(2.90, 7.0);
            // This pose is exactly 6 ft from the edge of the robot to the center of the edge of the subwoofer
            Translation2d W2_BLUE = new Translation2d(1.8288, 5.55);
            Translation2d W3_BLUE = new Translation2d(2.90-0.1, 4.10);

            Translation2d C1 = new Translation2d(8.29, 7.45);
            Translation2d C2 = new Translation2d(8.29, 5.78);
            // This pose is exactly 8 ft from the edge of the robot to the center of the edge of the center stage tag
            Translation2d C3 = new Translation2d(8.19, 4.10);
            Translation2d C4 = new Translation2d(8.29, 2.44);
            Translation2d C5 = new Translation2d(8.29, 0.77);

            Translation2d W1_RED = GeometryUtil.flipFieldPosition(W1_BLUE);
            Translation2d W2_RED = GeometryUtil.flipFieldPosition(W2_BLUE);
            Translation2d W3_RED = GeometryUtil.flipFieldPosition(W3_BLUE);
            
            add(W1_BLUE);
            add(W2_BLUE);
            add(W3_BLUE);
            add(C1);
            add(C2);
            add(C3);
            add(C4);
            add(C5);
            add(W1_RED);
            add(W2_RED);
            add(W3_RED);
        }};

        public static final HashMap<Translation2d, String> CALIBRATION_POSE_MAP = new HashMap<Translation2d, String>() {{
            Translation2d W1_BLUE = new Translation2d(2.90, 7.0);
            // This pose is exactly 6 ft from the edge of the robot to the center of the edge of the subwoofer
            Translation2d W2_BLUE = new Translation2d(1.8288, 5.55);
            Translation2d W3_BLUE = new Translation2d(2.90-0.1, 4.10);

            Translation2d C1 = new Translation2d(8.29, 7.45);
            Translation2d C2 = new Translation2d(8.29, 5.78);
            // This pose is exactly 8 ft from the edge of the robot to the center of the edge of the center stage tag
            Translation2d C3 = new Translation2d(8.19, 4.10);
            Translation2d C4 = new Translation2d(8.29, 2.44);
            Translation2d C5 = new Translation2d(8.29, 0.77);

            Translation2d W1_RED = GeometryUtil.flipFieldPosition(W1_BLUE);
            Translation2d W2_RED = GeometryUtil.flipFieldPosition(W2_BLUE);
            Translation2d W3_RED = GeometryUtil.flipFieldPosition(W3_BLUE);

            put(W1_BLUE, "W1_BLUE");
            put(W2_BLUE, "W2_BLUE");
            put(W3_BLUE, "W3_BLUE");
            put(C1, "C1");
            put(C2, "C2");
            put(C3, "C3");
            put(C4, "C4");
            put(C5, "C5");
            put(W1_RED, "W1_RED");
            put(W2_RED, "W2_RED");
            put(W3_RED, "W3_RED");
        }};

        // 12.4ft from the speaker
        public static final Translation2d PODIUM_SHOT_SPOT = new Translation2d(2.55, 4.19);
        // 9.76ft from the speaker
        public static final Translation2d ORBIT_POSE = new Translation2d(2.884, 6.304);

        public static final List<Pose2d> SHOOTING_POSITIONS = new ArrayList<Pose2d>() {{
            // We only want the blue alliance speaker translation since
            // it gets mirrored for the red alliance using GeometryUtil.flipFieldPose
            Translation2d BLUE_SPEAKER_TRANSLATION = SPEAKER_POSITIONS.get(0).getTranslation();
            Pose2d L_POSE2D = new Pose2d(L_POSE, new Rotation2d(
              BLUE_SPEAKER_TRANSLATION.getX() - L_POSE.getX(), 
              BLUE_SPEAKER_TRANSLATION.getY() - L_POSE.getY()
            ));
            Pose2d R_POSE2D = new Pose2d(R_POSE, new Rotation2d(
              BLUE_SPEAKER_TRANSLATION.getX() - R_POSE.getX(),
              BLUE_SPEAKER_TRANSLATION.getY() - R_POSE.getY()
            ));
            Pose2d M_POSE2D = new Pose2d(M_POSE, new Rotation2d(
              BLUE_SPEAKER_TRANSLATION.getX() - M_POSE.getX(), 
              BLUE_SPEAKER_TRANSLATION.getY() - M_POSE.getY()
            ));
            Pose2d W3_POSE2D = new Pose2d(W3_POSE, new Rotation2d(
              BLUE_SPEAKER_TRANSLATION.getX() - W3_POSE.getX(), 
              BLUE_SPEAKER_TRANSLATION.getY() - W3_POSE.getY()
            ));
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

        // Within a range of the [red circle](https://www.desmos.com/calculator/cu3ocssv5d)
        public static final double AUTOMATIC_SHOOTER_DISTANCE_RADIUS = 8.5;

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

        public static List<Pose2d> GET_ALL_NOTES() {
            List<Pose2d> notePoses = new ArrayList<Pose2d>();
            for (int i = 0; i < NOTE_TRANSLATIONS.length; i++) {
                notePoses.add(
                    new Pose2d(
                        NOTE_TRANSLATIONS[i].toTranslation2d(), 
                        new Rotation2d()));
            }
            return notePoses;
        }
    }

    public static final class CameraConstants {
        /**
         * Things to do when we are in field calibration
         * First off: Take snapshots at W2 (both alliances), and C3
         * 
         * Then, attempt to quickly make the fmap and verify the accuracy using a hotspot
         * (look up "april tag map builder limelight") or visit https://tools.limelightvision.io/map-builder
         * Then, go back and take another shot from either W1 or W3 to verify the accuracy
         * 
         * Finally, move the robot around and see if the vision helps us out...
         * Power cycle the limelight to force megatag to stop giving us outdated readings
         * 
         * Make sure to bring:
         *   Tape measure to verify the accuracy (Sub is 36 in out from field wall)
         *   Square (to make sure the robot is perfectly in the center of the spike mark)
         *   Phone for hotspot i suppose
         */
        public static final boolean FIELD_CALIBRATION_MODE = false;

        public static final double LL2_HORIZONTAL_FOV = 59.6;
        public static final double LL2_VERTICAL_FOV = 49.7;
        public static final double LL3_HORIZONTAL_FOV = 63.3;
        public static final double LL3_VERTICAL_FOV = 49.7;

        public static final long LIMELIGHT_MAX_UPDATE_TIME = 200_000; // Micro Seconds = 0.2 Seconds

        public static final Pose3d LL3Pose = 
            new Pose3d(
                // forward positive, right positive, up positive
                0.204,
                -0.234,
                0.651,
                new Rotation3d(0, Units.degreesToRadians(13), 0));
        public static final Pose3d LL2Pose =
            new Pose3d(
                // forward positive, right positive, up positive
                -0.272724,
                -0.311903,
                0.433974,
                // Pitch angle is measure from the horizontal
                // it is negative because it points down
                new Rotation3d(0, Units.degreesToRadians(-11), Units.degreesToRadians(180)));

        public static final Pose3d[] cameras = new Pose3d[] {LL3Pose, LL2Pose};
    }
    
    public static final class NTConstants {
        public static final int PIVOT_INDEX = 0;
        public static final int AMPPER_INDEX = 1;
        public static final int ELEVATOR_INDEX = 2;
        public static final int RIGHT_CLIMB_INDEX = 3;
        public static final int LEFT_CLIMB_INDEX = 4;
        
        public static final double PIVOT_OFFSET_X = -0.079531;
        public static final double PIVOT_OFFSET_Z = 0.178995;
    
        public static final Translation3d PIVOT_OFFSET_METERS = new Translation3d(
            PIVOT_OFFSET_X,
            0, 
            PIVOT_OFFSET_Z);
        
        public static final Map<String, Number> WAIT_TIMES = new HashMap<>() {{
            put("noteToShoot1", 0.05);
            put("noteToShoot2", 0.3);
            put("noteToShoot3", 0.0);

            put("intakeToTrap1", 0.2);
            put("intakeToTrap2", 0.0);

            put("noteToIndexer1", 0.2);
            put("noteToIndexer2", 0.07);
            put("noteToIndexer3", 0.0);

            put("noteToTrap1", 0.15);
            put("noteToTrap2", 0.254);
            put("noteToTrap3", 0.0);
            
            put("dropPiece1", 0.5);

            put("ampPosition", 0.37);
            put("atan++", 0.86);

            put("placeOuttake", 0.6995);
            put("prepPiece", 0.3);
        }};
    }
}