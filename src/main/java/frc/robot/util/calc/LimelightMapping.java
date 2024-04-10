package frc.robot.util.calc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.calc.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.calc.LimelightHelpers.Results;
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

    private Translation2d currentCalibrationPosition;
    private SwerveDrivePoseEstimator poseEstimator;
    Supplier<Pose2d> robotPoseSupplier;

    @Log
    private Pose3d[] modifiedTagPoses = new Pose3d[16];

    public LimelightMapping(SwerveDrivePoseEstimator poseEstimator, Supplier<Pose2d> robotPoseSupplier, String limelightName) {
        currentCalibrationPosition = FieldConstants.TAG_CALIBRATION_POSE.get(0);
        this.robotPoseSupplier = robotPoseSupplier;
        this.poseEstimator = poseEstimator;
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
        int currentIndex = FieldConstants.TAG_CALIBRATION_POSE.indexOf(currentCalibrationPosition);
        // modulus wrap our index so we stay in bounds
        if (currentIndex == FieldConstants.TAG_CALIBRATION_POSE.size() - 1) {
            currentCalibrationPosition = FieldConstants.TAG_CALIBRATION_POSE.get(0);
        } else {
            currentCalibrationPosition = FieldConstants.TAG_CALIBRATION_POSE.get(currentIndex + 1);
        }
        System.out.println("Set calibration pose to " + FieldConstants.CALIBRATION_POSE_MAP.get(currentCalibrationPosition));
    }

    public void decrementCalibrationPose() {
        int currentIndex = FieldConstants.TAG_CALIBRATION_POSE.indexOf(currentCalibrationPosition);
        // modulus wrap our index so we stay in bounds
        if (currentIndex == 0) {
            currentCalibrationPosition = FieldConstants.TAG_CALIBRATION_POSE.get(FieldConstants.TAG_CALIBRATION_POSE.size() - 1);
        } else {
            currentCalibrationPosition = FieldConstants.TAG_CALIBRATION_POSE.get(currentIndex - 1);
        }
        System.out.println("Set calibration pose to " + FieldConstants.CALIBRATION_POSE_MAP.get(currentCalibrationPosition));
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

    public Translation2d getCurrentCalibrationPosition() {
        return currentCalibrationPosition;
    }

    public void takeSnapshot() {
        Translation2d actualCalibrationRobotPose = currentCalibrationPosition;
        LimelightTarget_Fiducial[] recordedPoses = LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
        for (int i = 0; i < recordedPoses.length; i++) {
            int fiducialID = (int) recordedPoses[i].fiducialID;

            Pose3d targetPose = targetPoseFieldSpace(
                recordedPoses[i].getTargetPose_RobotSpace(), 
                new Pose3d(new Pose2d(actualCalibrationRobotPose, robotPoseSupplier.get().getRotation()))
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
                0,
                0, 
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

    public Command updatePoseEstimatorCommand() {
        return Commands.run(
            () -> {
                this.updatePoseEstimator();
            }
        );
    }

    @Log
    Pose2d estimatedPose2d = new Pose2d();

    private void updatePoseEstimator() {
        if (LimelightHelpers.getCurrentPipelineIndex(limelightName) != 0) {
            LimelightHelpers.setPipelineIndex(limelightName, 0);
        }
        Results result = getResults();
        Pose2d estimatedRobotPose = result.getBotPose2d_wpiBlue();

        LimelightTarget_Fiducial[] targets = result.targets_Fiducials;      

        // invalid data check
        if (estimatedRobotPose.getX() == 0.0
            || Double.isNaN(estimatedRobotPose.getX()) 
            || Double.isNaN(estimatedRobotPose.getY()) 
            || Double.isNaN(estimatedRobotPose.getRotation().getRadians())
            || targets.length == 0
            || Double.valueOf(targets[0].tx).equals(null)
            || Double.valueOf(targets[0].ty).equals(null)
            || !Double.isFinite(targets[0].tx)
            || !Double.isFinite(targets[0].ty))
                return;
             

        if (hasTarget(result)) {
            double xyStds;
            double radStds;
            // multiple targets detected
            if (targets.length > 1) {
                // TODO: TUNE
                xyStds = 0.8;
                radStds = 1.2;
            }
            // 1 target with large area and close to estimated pose
            else if (LimelightHelpers.getTA(limelightName) > 0.175) {
                // TODO: TUNE
                xyStds = 1.6;
                radStds = 4;
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }

            this.estimatedPose2d = estimatedRobotPose;

            poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, radStds));
            poseEstimator.addVisionMeasurement(estimatedRobotPose,
                Timer.getFPGATimestamp() - getLatencyDiffSeconds(result));
        }
    }

    public Results getResults() {
        return LimelightHelpers.getLatestResults(limelightName).targetingResults;
    }

    public double getLatencyDiffSeconds() {
        return (LimelightHelpers.getLatency_Pipeline(limelightName)/1000d) - (LimelightHelpers.getLatency_Capture(limelightName)/1000d); 
    }

    public double getLatencyDiffSeconds(Results result) {
        return (result.latency_pipeline/1000.0) - (result.latency_capture/1000.0); 
    }

    public boolean hasTarget(Results result) {
        if (result == null || !result.valid || (result.botpose[0] == 0 && result.botpose[1] == 0)) {
            return false;
        }

        if ((LimelightHelpers.getTA(limelightName) < 0.175 && result.targets_Fiducials.length == 1)
            || (result.targets_Fiducials.length > 1 && LimelightHelpers.getTA(limelightName) < 0.15))
        {
            return false;
        }
      
        return true;
    }

    private boolean isWithinTolerance(Pose3d existingPose, Pose3d targetPose) {
        return MathUtil.isNear(
            0, 
            existingPose.getTranslation().getDistance(targetPose.getTranslation()), 
            Units.inchesToMeters(1.5)
        );
    }

}