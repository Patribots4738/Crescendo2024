package frc.robot.util.calc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.CameraConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.calc.LimelightHelpers.LimelightTarget_Fiducial;
import monologue.Logged;
import monologue.Annotations.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import org.json.simple.JSONObject;

public class LimelightMapping extends SubsystemBase implements Logged {
    private LimelightConversion limelightConversion;
    private HashMap<Integer, Pose3d> poses;
    private String limelightName;

    private Translation2d currentCalibrationPose;
    private Supplier<Rotation2d> rotationSupplier;

    @Log
    private Pose3d[] modifiedTagPoses = new Pose3d[16];

    public LimelightMapping(String limelightName, Supplier<Rotation2d> rotationSupplier) {
        currentCalibrationPose = FieldConstants.TAG_CALIBRATION_POSE.get(0);
        this.rotationSupplier = rotationSupplier;
        this.limelightName = limelightName;
        poses = new HashMap<>();
        limelightConversion = new LimelightConversion();

        // Generate empty poses so that it gets logged
        for (int i = 0; i < modifiedTagPoses.length; i++) {
            modifiedTagPoses[i] = new Pose3d();
        }
    }

    public void addAll() {
        modifiedTagPoses = poses.keySet().toArray(new Pose3d[0]);
        limelightConversion.addAll(modifiedTagPoses);
    }

    public void incrementCalibrationPose() {
        int currentIndex = FieldConstants.TAG_CALIBRATION_POSE.indexOf(currentCalibrationPose);
        // modulus wrap our index so we stay in bounds
        if (currentIndex == FieldConstants.TAG_CALIBRATION_POSE.size() - 1) {
            currentCalibrationPose = FieldConstants.TAG_CALIBRATION_POSE.get(0);
        } else {
            currentCalibrationPose = FieldConstants.TAG_CALIBRATION_POSE.get(currentIndex + 1);
        }
        System.out.println("Set calibration pose to " + FieldConstants.CALIBRATION_POSE_MAP.get(currentCalibrationPose));
    }

    public void decrementCalibrationPose() {
        int currentIndex = FieldConstants.TAG_CALIBRATION_POSE.indexOf(currentCalibrationPose);
        // modulus wrap our index so we stay in bounds
        if (currentIndex == 0) {
            currentCalibrationPose = FieldConstants.TAG_CALIBRATION_POSE.get(FieldConstants.TAG_CALIBRATION_POSE.size() - 1);
        } else {
            currentCalibrationPose = FieldConstants.TAG_CALIBRATION_POSE.get(currentIndex - 1);
        }
        System.out.println("Set calibration pose to " + FieldConstants.CALIBRATION_POSE_MAP.get(currentCalibrationPose));
    }

    public Command incrementCalibrationPose(boolean increment) {
        return Commands.runOnce(() -> {
            if (increment) {
                incrementCalibrationPose();
            } else {
                decrementCalibrationPose();
            }
        }).ignoringDisable(true);
    }

    public Translation2d getCurrentCalibrationPose() {
        return currentCalibrationPose;
    }

    public void takeSnapshot() {
        Pose2d actualRobotPose = new Pose2d(currentCalibrationPose, rotationSupplier.get());
        LimelightTarget_Fiducial[] recordedPoses = LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
        for (int i = 0; i < recordedPoses.length; i++) {
            int fiducialID = (int) recordedPoses[i].fiducialID;

            Pose3d targetPose = targetPoseFieldSpace(
                recordedPoses[i].getTargetPose_RobotSpace(), 
                new Pose3d(actualRobotPose)
            );

            // Check if the value is within tolerance
            if (poses.containsKey(fiducialID)) {
                Pose3d existingPose = poses.get(fiducialID);
                if (isWithinTolerance(existingPose, targetPose)) {
                    System.out.println("Tag " + fiducialID + " is within tolerance, skipping");
                    continue; // Skip overwriting
                } else {
                    System.out.println("Tag " + fiducialID + " is not within tolerance, overwriting");
                }
            }
            modifiedTagPoses[fiducialID-1] = targetPose;
            poses.put(fiducialID, targetPose);
            limelightConversion.addFiducial(fiducialID, targetPose);
        }
    }

    public void ManuallyAddPose(int fiducialID, Pose3d targetPose) {
        limelightConversion.addFiducial(fiducialID, targetPose);
        poses.put(fiducialID, targetPose);
    }

    public Command takeSnapshotCommand() {
        return Commands.runOnce(this::takeSnapshot)
            .andThen(Commands.print("Snapshot taken"))
            .ignoringDisable(true);
    }

    public Pose3d targetPoseFieldSpace(Pose3d targetPose, Pose3d actualRobotPose) {
        targetPose = new Pose3d(
            targetPose.getZ(), 
            -targetPose.getX(), 
            -targetPose.getY(), 
            new Rotation3d(
                targetPose.getRotation().getX(), 
                targetPose.getRotation().getY(), 
                Math.PI+targetPose.getRotation().getZ()
            )
        );
        Pose2d rotatedPose = targetPose.toPose2d().rotateBy(actualRobotPose.getRotation().toRotation2d());
        Pose3d position = 
            new Pose3d(
                rotatedPose.getX(), 
                rotatedPose.getY(), 
                targetPose.getZ(), 
                new Rotation3d()
            )
            .plus(
                new Transform3d(
                    actualRobotPose.getTranslation(),
                    actualRobotPose.getRotation()
                )
            );
        return new Pose3d(position.getTranslation(), targetPose.getRotation().rotateBy(actualRobotPose.getRotation()));
    }

    public void printJSON() {
        Pose3d[] poseArray = new Pose3d[poses.size()];
        for (int i = 0; i < poseArray.length; i++) {
            poseArray[i] = poses.get(i);
        }
        JSONObject[] jsonData = limelightConversion.tagMap.get("fiducials");
        ArrayList<JSONObject> newJsonData = new ArrayList<>();
        // remove any null values so then its easier to auto import to limelight visualizer
        for (int i = 0; i < jsonData.length; i++) {
            if(jsonData[i] == null) continue;
            newJsonData.add(jsonData[i]);
        }
        System.out.println("\n\n\n{");
        System.out.println(JSONObject.toString("fiducials", newJsonData.toArray(new JSONObject[0])));
        System.out.println("}\n\n\n");
    }

    public Command printJSONCommand() {
        return Commands.runOnce(this::printJSON).ignoringDisable(true);
    }

    private boolean isWithinTolerance(Pose3d existingPose, Pose3d targetPose) {
        return MathUtil.isNear(
            0, 
            existingPose.getTranslation().getDistance(targetPose.getTranslation()), 
            Units.inchesToMeters(1.5)
        );
    }

}