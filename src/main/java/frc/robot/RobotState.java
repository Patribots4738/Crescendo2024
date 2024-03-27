package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.NTConstants;
import frc.robot.Constants.ShooterConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class RobotState implements Logged{
    @Log
    public static Pose3d[] components3d = new Pose3d[5];
    @Log
    public static Pose3d[] desiredComponents3d = new Pose3d[5];
    @Log
    public static Pose3d[] notePose3ds = new Pose3d[12];
    @Log
    public static Pose3d[] highNotePose3ds = new Pose3d[12];
    @Log
    public static boolean freshCode = true;
    @Log
    public static Field2d field2d = new Field2d();
    @Log
    public static Pose2d robotPose2d = new Pose2d();
    @Log
    public static Transform2d visionErrorPose = new Transform2d();
    @Log
    public static double distanceToSpeakerMeters = 0;
    @Log
    public static Pose3d robotPose3d = new Pose3d();
    @Log
    public static SwerveModuleState[] swerveMeasuredStates;
    @Log
    public static SwerveModuleState[] swerveDesiredStates;
    @Log
    public static double gameModeStart = 0;
    @Log
    public static boolean enableVision = true;

    private static final Pose3d INITIAL_SHOOTER_POSE = new Pose3d(
                NTConstants.PIVOT_OFFSET_METERS.getX(),
                0,
                NTConstants.PIVOT_OFFSET_METERS.getZ(),
            new Rotation3d(0, -Units.degreesToRadians(ShooterConstants.PIVOT_LOWER_LIMIT_DEGREES), 0)
    );
        
    public RobotState() {
    }

    private static void setInitialComponents3d() {
        components3d[0] = INITIAL_SHOOTER_POSE;
    }

    private static void setInitialDesiredComponents3d() {
        desiredComponents3d[0] = INITIAL_SHOOTER_POSE;
    }

    private static void setInitialNotePose3ds() {
        notePose3ds[0] = new Pose3d();
    }

    private static void setInitialHighNotePose3ds() {
        highNotePose3ds[0] = new Pose3d(0, 0, -0.1, new Rotation3d());
    }

    private static void initAllComponents3d() {
        setInitialComponents3d();
        setInitialDesiredComponents3d();
        for (int i = 1; i < components3d.length; i++) {
            components3d[i] = new Pose3d();
            desiredComponents3d[i] = new Pose3d();
        }
    }

    private static void initAllNotePose3ds() {
        setInitialNotePose3ds();
        setInitialHighNotePose3ds();
        for (int i = 1; i < notePose3ds.length; i++) {
            notePose3ds[i] = new Pose3d(FieldConstants.NOTE_TRANSLATIONS[i-1], new Rotation3d());
            highNotePose3ds[i] = new Pose3d(FieldConstants.HIGH_NOTE_TRANSLATIONS[i-1], new Rotation3d());
        }
    }

    public static void initComponents() {
        initAllComponents3d();
        initAllNotePose3ds();
    }

    public static void startCode() {
        freshCode = true;
    }

    public static Command enableVision() {
        return Commands.runOnce(() -> RobotState.enableVision = true).ignoringDisable(true);
    }

    public static Command disableVision() {
        return Commands.runOnce(() -> RobotState.enableVision = false).ignoringDisable(true);
    }

}
