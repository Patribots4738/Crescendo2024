package frc.robot.util.calc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Limelight;
import frc.robot.util.calc.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.custom.LimelightConversion;
import java.util.List;

import com.pathplanner.lib.util.GeometryUtil;

import java.util.ArrayList;
import java.util.HashMap;

public class LimelightMapping {
    private LimelightConversion limelightConversion;
    private HashMap<Integer, Pose3d> poses;
    private String limelightName;

    public LimelightMapping(String limelightName) {
        this.limelightName = limelightName;
        poses = new HashMap<>();
        limelightConversion = new LimelightConversion();
    }

    public void addAll() {
        Pose3d[] poseArray = poses.keySet().toArray(new Pose3d[0]);
        limelightConversion.addAll(poseArray);
    }

    public void addLiveTagPoses(Pose2d calibrationPose) {
        LimelightTarget_Fiducial[] recordedPoses = LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
        for (int i = 0; i < recordedPoses.length; i++) {
            int fiducialID = (int) recordedPoses[i].fiducialID;
            Pose3d targetPose = recordedPoses[i].getTargetPose_RobotSpace()
                .plus(new Transform3d(
                    calibrationPose.getX(),
                    calibrationPose.getY(),
                    0,
                    new Rotation3d()
                ));

            // Check if the value is within tolerance
            if (poses.containsKey(fiducialID)) {
                Pose3d existingPose = poses.get(fiducialID);
                if (isWithinTolerance(existingPose, targetPose)) {
                    continue; // Skip overwriting
                }
            }

            poses.put(fiducialID, targetPose);
        }
    }

    private boolean isWithinTolerance(Pose3d existingPose, Pose3d targetPose) {
        return MathUtil.isNear(existingPose.getTranslation().getX(), targetPose.getTranslation().getX(), Units.inchesToMeters(0.15)) &&
            MathUtil.isNear(existingPose.getTranslation().getY(), targetPose.getTranslation().getY(), Units.inchesToMeters(0.15)) &&
            MathUtil.isNear(existingPose.getTranslation().getZ(), targetPose.getTranslation().getZ(), Units.inchesToMeters(0.15));
    }

}