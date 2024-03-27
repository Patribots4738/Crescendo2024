package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot.GameMode;
import lib.limelightHelpers.LimelightHelpers;
import lib.limelightHelpers.LimelightHelpers.LimelightTarget_Detector;
import lib.limelightHelpers.LimelightHelpers.LimelightTarget_Fiducial;
import lib.limelightHelpers.LimelightHelpers.Results;
import monologue.Logged;
import monologue.Annotations.Log;

// https://github.com/NAHSRobotics-Team5667/2020-FRC/blob/master/src/main/java/frc/robot/utils/LimeLight.java
public class Limelight extends SubsystemBase implements Logged{

    String limelightName;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final SwerveDrivePoseEstimator poseEstimator;

    @Log
    Pose3d[] visableTags;

    private static NetworkTableEntry timingTestEntry;
    private static boolean timingTestEntryValue = false;
    private int pipelineIndex;

    @Log
    public boolean isConnected = false;
    
    @Log
    private Pose2d estimatedPose2d = new Pose2d();

    @Log
    public long timeDifference = 999_999; // Micro Seconds = 0.999999 Seconds | So the limelight is not connected if the time difference is greater than LimelightConstants.LIMELIGHT_MAX_UPDATE_TIME

    @Log
    private Pose2d notePose2d = new Pose2d();

    public Limelight(SwerveDrivePoseEstimator poseEstimator, Supplier<Pose2d> robotPoseSupplier, String limelightName, int pipelineIndex) {
        // Uses network tables to check status of limelight
        this.limelightName = limelightName;
        this.pipelineIndex = pipelineIndex;
        timingTestEntry = LimelightHelpers.getLimelightNTTableEntry(limelightName, "TIMING_TEST_ENTRY");
        this.robotPoseSupplier = robotPoseSupplier;
        this.poseEstimator = poseEstimator;
        loadAprilTagFieldLayout();
        setPipelineIndex(pipelineIndex);
    }

    @Override
    public void periodic() {
        if (FieldConstants.IS_SIMULATION) {
            updateCameras(robotPoseSupplier.get());
        } else {
            if (pipelineIndex == 0) {
                updatePoseEstimator();
                getTags();
            } else if (pipelineIndex == 1) {
                notePose2d = getNotePose2d();
            }
        }
    }

    private void updatePoseEstimator() {
        if (LimelightHelpers.getCurrentPipelineIndex(limelightName) != 0) {
            LimelightHelpers.setPipelineIndex(limelightName, 0);
        }
        Results result = getResults();
        Pose2d estimatedRobotPose = result.getBotPose2d_wpiBlue();

        LimelightTarget_Fiducial[] targets = result.targets_Fiducials;

        this.estimatedPose2d = estimatedRobotPose;
        RobotState.visionErrorPose = RobotState.robotPose2d.minus(estimatedRobotPose);
        double xyStds;
        double radStds;
        if (Robot.gameMode == GameMode.DISABLED || 
            (Robot.gameMode == GameMode.AUTONOMOUS 
                && Robot.currentTimestamp - RobotState.gameModeStart < 3 && hasTarget(result))) {
            xyStds = 0.001;
            radStds = 0.0002;
            
            poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, radStds));
            poseEstimator.addVisionMeasurement(estimatedRobotPose,
                Timer.getFPGATimestamp() - getLatencyDiffSeconds(result));
        }
        else if (RobotState.enableVision && hasTarget(result)) {
            // multiple targets detected
            if (targets.length > 1) {
                if (Robot.gameMode == GameMode.TELEOP) {
                    // Trust the vision even more
                    if (targets.length > 2) {
                        xyStds = Math.hypot(0.004, 0.002);
                    } else {
                        // We can only see two tags, it's trustable still
                        xyStds = Math.hypot(0.01, 0.005);
                    }
                } else {
                    xyStds = Math.hypot(0.011, 0.028);
                }
                radStds = Units.degreesToRadians(2);
            }
            // 1 target with large area and close to estimated roxose
            else if (LimelightHelpers.getTA(limelightName) > 0.175) {
                xyStds = 0.192;
                radStds = Units.degreesToRadians(7);
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }

            poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, radStds));
            poseEstimator.addVisionMeasurement(estimatedRobotPose,
                Timer.getFPGATimestamp() - getLatencyDiffSeconds(result));
        }
    }

    public Results getResults() {
        return LimelightHelpers.getLatestResults(limelightName).targetingResults;
    }

    public void setPipelineIndex(int index) {
        LimelightHelpers.setPipelineIndex(limelightName, index);
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

    @Log
    Pose2d noteFieldPose = new Pose2d();

    @Log
    Pose2d noteFieldPosePlus14 = new Pose2d();

    public Pose2d getNotePose2d() {
        Translation2d noteTranslationFromRobot;
        Pose2d nextNoteFieldPose = noteFieldPose;
        if (FieldConstants.IS_SIMULATION) {
            noteFieldPose = robotPoseSupplier.get().nearest(FieldConstants.GET_CENTERLINE_NOTES());
            noteTranslationFromRobot = noteFieldPose.relativeTo(robotPoseSupplier.get()).getTranslation();
        } else {
            if (LimelightHelpers.getCurrentPipelineIndex(limelightName) != 1) {
                LimelightHelpers.setPipelineIndex(limelightName, 1);
            }

            Results results = getResults();

            if (noteInVision(results)) {
                for (LimelightTarget_Detector ld : results.targets_Detector) {
                    ld.calculateYDistance(CameraConstants.LL2Pose.getZ(), CameraConstants.LL2Pose.getRotation().getY());
                    ld.calculateXDistance(CameraConstants.LL2Pose.getRotation().toRotation2d().getRadians());
                }

                Translation2d noteTranslationFromCamera = new Translation2d(
                    results.targets_Detector[0].calcY,
                    -results.targets_Detector[0].calcX
                );

                noteTranslationFromRobot = noteTranslationFromCamera
                    .plus(CameraConstants.LL2Pose.getTranslation().toTranslation2d())
                    .rotateBy(CameraConstants.LL2Pose.getRotation().toRotation2d());

                nextNoteFieldPose = robotPoseSupplier.get().plus(new Transform2d(noteTranslationFromRobot, new Rotation2d()));
            } else {
                return noteFieldPosePlus14;
            }
        }

        Rotation2d slopeAngle = new Rotation2d(
            noteTranslationFromRobot.getX(),
            noteTranslationFromRobot.getY()
        ).plus(Rotation2d.fromDegrees(180));

        double distanceThreshold = FieldConstants.IS_SIMULATION ? Units.inchesToMeters(18) : Units.inchesToMeters(20);
        if ((noteFieldPose.relativeTo(robotPoseSupplier.get()).getTranslation().getNorm() > distanceThreshold)
            || noteFieldPose.getTranslation().getDistance(nextNoteFieldPose.getTranslation()) > Units.inchesToMeters(3)) {

            noteFieldPose = nextNoteFieldPose;
            
            noteFieldPosePlus14 = noteFieldPose.plus(
                new Transform2d(
                    new Translation2d(
                        Units.inchesToMeters(9.5),
                        slopeAngle.plus(Rotation2d.fromDegrees(180))
                    ),
                    slopeAngle
                )
            );
        }

        return noteFieldPosePlus14;
    }

    public boolean noteInVision(Results results) {
        return (FieldConstants.IS_SIMULATION && ((int) Robot.currentTimestamp/3) % 2 == 0) ||
            (results.valid 
            && results.targets_Detector.length > 0 
            && LimelightHelpers.getTA(limelightName) > 0.5 
            && results.targets_Detector[0].tx != 0 
            && results.targets_Detector[0].ty != 0);
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

    public double getLatencyDiffSeconds(Results result) {
        return (result.latency_pipeline/1000.0) - (result.latency_capture/1000.0); 
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

    @Log
    int tagsViable = 0;

    public boolean hasTarget(Results result) {

        Pose2d estimatedRobotPose = result.getBotPose2d_wpiBlue();
        LimelightTarget_Fiducial[] targets = result.targets_Fiducials;

        double singleTagAmbiguityThreshold = Robot.gameMode == GameMode.AUTONOMOUS ? 0.175 : 0.175;
        double multiTagAmbiguityThreshold = Robot.gameMode == GameMode.AUTONOMOUS ? 0.13 : 0.07;
        if (result == null || !result.valid 
            || (LimelightHelpers.getTA(limelightName) < singleTagAmbiguityThreshold && result.targets_Fiducials.length == 1)
            || (result.targets_Fiducials.length > 1 && LimelightHelpers.getTA(limelightName) < multiTagAmbiguityThreshold)
            || (estimatedRobotPose.getX() == 0 && estimatedRobotPose.getY() == 0)
            || Double.isNaN(estimatedRobotPose.getX()) 
            || Double.isNaN(estimatedRobotPose.getY()) 
            || Double.isNaN(estimatedRobotPose.getRotation().getRadians())
            || targets.length == 0
            || Double.valueOf(targets[0].tx).equals(null)
            || Double.valueOf(targets[0].ty).equals(null)
            || !Double.isFinite(targets[0].tx)
            || !Double.isFinite(targets[0].ty))
        {
            return false;
        }
      
        return true;
    }

    // https://github.com/StuyPulse/Alfred/blob/c7ebcdf0e586a32e6e28b5b808fb6aee6deee325/src/main/java/frc/util/Limelight.java#L28
    public boolean isConnected() {
        // Force an update and get current time
        timingTestEntryValue = !timingTestEntryValue; // flip test value
        timingTestEntry.setBoolean(timingTestEntryValue);
        long currentTime = timingTestEntry.getLastChange();

        // Get most recent update from limelight
        long lastUpdate = LimelightHelpers.getLimelightNTTableEntry(limelightName, "tl").getLastChange();

        // Calculate limelights last update
        timeDifference = currentTime - lastUpdate;
        isConnected = timeDifference < CameraConstants.LIMELIGHT_MAX_UPDATE_TIME;

        return isConnected;
    }

    public Pose2d getRobotPoseTargetSpace() {
        return LimelightHelpers.getBotPose3d_TargetSpace(limelightName).toPose2d();
    }

    public Command blinkLeds(DoubleSupplier duration) {
        return Commands.sequence(
            runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink(limelightName)),
            Commands.waitSeconds(duration.getAsDouble()),
            runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(limelightName))
        ).ignoringDisable(true).asProxy();
    }

    public Command setLEDState(BooleanSupplier enabled) {
        return Commands.either(
                Commands.runOnce(this::enableLEDS), 
                Commands.runOnce(this::disableLEDS), 
                enabled
            ).ignoringDisable(true).asProxy();
    }

    public void enableLEDS() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    public void disableLEDS() {
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
    }
}