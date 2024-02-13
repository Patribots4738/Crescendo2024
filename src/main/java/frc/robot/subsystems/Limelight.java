package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Constants.CameraConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.LimelightHelpers.Results;
import monologue.Logged;
import monologue.Annotations.Log;

// https://github.com/NAHSRobotics-Team5667/2020-FRC/blob/master/src/main/java/frc/robot/utils/LimeLight.java
public class Limelight extends SubsystemBase implements Logged{

    String limelightName = "limelight";
    private final Supplier<Pose2d> robotPoseSupplier;

    @Log
    Pose3d[] visableTags;

    @Log
    public boolean isConnected = false;

    @Log
    public long timeDifference = 999_999; // Micro Seconds = 0.999999 Seconds | So the limelight is not connected if the time difference is greater than LimelightConstants.LIMELIGHT_MAX_UPDATE_TIME


    public Limelight(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        loadAprilTagFieldLayout();
    }

    @Override
    public void periodic() {
        if (FieldConstants.IS_SIMULATION) {
            updateCameras(robotPoseSupplier.get());
        } else {
            getTags();
        }
    }

    public Results getResults() {
        return LimelightHelpers.getLatestResults(limelightName).targetingResults;
    }

    private LimelightTarget_Fiducial[] getTags() {
        LimelightTarget_Fiducial[] fiducials = LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
        
        setFiducialPoses(fiducials);

        return fiducials;
    }

    private void setFiducialPoses(LimelightTarget_Fiducial[] fiducials) {
        ArrayList<Pose3d> knownFiducials = new ArrayList<>();

        for (LimelightTarget_Fiducial target : fiducials) {
            int tagID = (int) target.fiducialID;
            if (tagID < aprilTagFieldLayout.getTags().size()) {
                knownFiducials.add(aprilTagFieldLayout.getTagPose(tagID).get());
            }
        }

        visableTags = knownFiducials.toArray(new Pose3d[0]);
    }

    public Pose2d getPose2d() {
        return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
    }

    public double getTagID() {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public double getLatencyDiffSeconds() {
        return (LimelightHelpers.getLatency_Pipeline(limelightName)/1000d) - (LimelightHelpers.getLatency_Capture(limelightName)/1000d); 
    }

    // The below code is for simulation only
    // This has nothing to do with the actual limelight

    private AprilTagFieldLayout aprilTagFieldLayout;

    /**
     * Loads the AprilTag field layout from a resource file.
     * If the file is not found, an IOException is caught and a message is printed.
     */
    public void loadAprilTagFieldLayout() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("AprilTag field layout not found! What year is it again?");
        }
    }
    
    /**
     * Updates the cameras by checking if any poses are in the field of view of the cameras.
     * 
     * @param robotPose the pose of the robot
     */
    public void updateCameras(Pose2d robotPose) {
        // Loop through the tag field layout to see if any poses are in the field of view of our cameras
        // Also check to see if the distance is 
        ArrayList<Pose3d> poses = new ArrayList<Pose3d>();  
        for (int i = 0; i < aprilTagFieldLayout.getTags().size(); i++) {
            AprilTag tag = aprilTagFieldLayout.getTags().get(i);
            for (int j = 0; j < CameraConstants.cameras.length; j++) {
                if (isInFieldOfView(robotPose, CameraConstants.cameras[j], tag.pose)) {
                    poses.add(tag.pose); 
                }
            }
        }
        visableTags = poses.toArray(new Pose3d[0]);
    }

    /**
     * Checks if a given tag pose is within the field of view of the camera, based
     * on the robot's pose and camera's pose.
     * 
     * @param robotPose  The pose of the robot.
     * @param cameraPose The pose of the camera.
     * @param tagPose    The pose of the tag.
     * @return True if the tag is within the field of view, false otherwise.
     */
    private boolean isInFieldOfView(Pose2d robotPose, Pose3d cameraPose, Pose3d tagPose) {
        // Put the camera on the robot
        cameraPose = new Pose3d(robotPose).plus(new Transform3d(cameraPose.getTranslation(), cameraPose.getRotation()));
        
        Pose3d cameraToTag = tagPose.relativeTo(cameraPose);
        // Get the atan of the camera's x and y
        // to see if it is in frame
        Rotation2d yawAngle = new Rotation2d(cameraToTag.getTranslation().getX(), cameraToTag.getTranslation().getY());
        // Check if the tag is facing the camera (roughly)
        boolean angleCheck = Math.abs(yawAngle.getDegrees()) < 45;
        boolean distanceCheck = cameraToTag.getTranslation().getX() < 3;
        boolean isFacing = Math.signum(tagPose.getRotation().toRotation2d().minus(cameraPose.getRotation().toRotation2d()).getDegrees()) == -1;
        
        return angleCheck && distanceCheck && isFacing;
    }
}