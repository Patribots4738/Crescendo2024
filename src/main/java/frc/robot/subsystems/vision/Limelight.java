package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.RobotContainer;
import frc.robot.Robot.GameMode;
import frc.robot.util.Constants.CameraConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.calc.LimelightHelpers;
import frc.robot.util.calc.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.util.calc.LimelightHelpers.Results;

// https://github.com/NAHSRobotics-Team5667/2020-FRC/blob/master/src/main/java/frc/robot/utils/LimeLight.java
public class Limelight extends SubsystemBase implements LimelightIO {

    String limelightName;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

    Pose3d[] visibleTags;

    private static NetworkTableEntry timingTestEntry;
    private static boolean timingTestEntryValue = false;

    public boolean isConnected = false;
    
    private Pose2d estimatedPose2d = new Pose2d();

    public long timeDifference = 999_999; // Micro Seconds = 0.999999 Seconds | So the limelight is not connected if the time difference is greater than LimelightConstants.LIMELIGHT_MAX_UPDATE_TIME

    private Pose2d notePose2d = new Pose2d();

    public Limelight(SwerveDrivePoseEstimator poseEstimator, Supplier<Pose2d> robotPoseSupplier, String limelightName, int pipelineIndex) {
        // Uses network tables to check status of limelight
        this.limelightName = limelightName;
        timingTestEntry = LimelightHelpers.getLimelightNTTableEntry(limelightName, "TIMING_TEST_ENTRY");
        this.robotPoseSupplier = robotPoseSupplier;
        this.poseEstimator = poseEstimator;
        loadAprilTagFieldLayout();
        setPipelineIndex(pipelineIndex);
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/" + limelightName, inputs);
        if (FieldConstants.IS_SIMULATION) {
            updateCameras(robotPoseSupplier.get());
        } else {
            updatePoseEstimator();
            setFiducialPoses();
        }
        Logger.recordOutput("Subsystems/" + limelightName + "/VisibleTags", visibleTags);
        Logger.recordOutput("Subsystems/" + limelightName + "/IsConnected", isConnected);
        Logger.recordOutput("Subsystems/" + limelightName + "/EstimatedPose2d", estimatedPose2d);
        Logger.recordOutput("Subsystems/" + limelightName + "/TimeDifference", timeDifference);
        Logger.recordOutput("Subsystems/" + limelightName + "/NotePose2d", notePose2d);
        Logger.recordOutput("Subsystems/" + limelightName + "/NoteFieldPose", noteFieldPose);
        Logger.recordOutput("Subsystems/" + limelightName + "/NoteFieldPosePlus14", noteFieldPosePlus14);
        Logger.recordOutput("Subsystems/" + limelightName + "/TagsViable", tagsViable);
    }

    private void updatePoseEstimator() {
        
        Pose2d estimatedRobotPose = inputs.botPose2d;
        
        this.estimatedPose2d = estimatedRobotPose;
        RobotContainer.visionErrorPose = RobotContainer.robotPose2d.minus(estimatedRobotPose);
        double xyStds;
        double radStds;
        if ((Robot.gameMode == GameMode.DISABLED || 
            Robot.gameMode == GameMode.AUTONOMOUS
                && Robot.currentTimestamp - RobotContainer.gameModeStart < 1.75)
                && hasTarget()) {
            xyStds = 0.001;
            radStds = 0.0002;
            
            poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, radStds));
            poseEstimator.addVisionMeasurement(estimatedRobotPose,
                Timer.getFPGATimestamp() - getLatencyDiffSeconds());
        }
        else if (RobotContainer.enableVision && hasTarget()) {
            // multiple targets detected
            if (inputs.targetIDs.length > 1) {
                if (Robot.gameMode == GameMode.TELEOP) {
                    // Trust the vision even MORE
                    if (inputs.targetIDs.length > 2) {
                        xyStds = Math.hypot(0.002, 0.003);
                    } else {
                        // We can only see two tags, (still trustable)
                        xyStds = Math.hypot(0.005, 0.008);
                    }
                } else {
                    xyStds = Math.hypot(0.014, 0.016);
                }
                radStds = Units.degreesToRadians(2);
            }
            // 1 target with large area and close to estimated roxose
            else if (inputs.limelightTA > 0.14) {
                xyStds = Math.hypot(0.015, 0.033);
                radStds = Units.degreesToRadians(7);
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }

            poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, radStds));
            poseEstimator.addVisionMeasurement(estimatedRobotPose,
                Timer.getFPGATimestamp() - getLatencyDiffSeconds());
        }
    }

    public Results getResults() {
        return LimelightHelpers.getLatestResults(limelightName).targetingResults;
    }

    public void setPipelineIndex(int index) {
        LimelightHelpers.setPipelineIndex(limelightName, index);
        inputs.pipelineIndex = index;
    }

    private void setFiducialPoses() {
        ArrayList<Pose3d> knownFiducials = new ArrayList<>();

        for (int targetID : inputs.targetIDs) {
            if (targetID < aprilTagFieldLayout.getTags().size()) {
                knownFiducials.add(aprilTagFieldLayout.getTagPose(targetID).get());
            }
        }

        visibleTags = knownFiducials.toArray(new Pose3d[0]);
    }

    Pose2d noteFieldPose = new Pose2d();

    Pose2d noteFieldPosePlus14 = new Pose2d();

    public Pose2d getNotePose2d() {
        Translation2d noteTranslationFromRobot;
        Pose2d nextNoteFieldPose = noteFieldPose;
        if (FieldConstants.IS_SIMULATION) {
            noteFieldPose = robotPoseSupplier.get().nearest(FieldConstants.GET_CENTERLINE_NOTES());
            noteTranslationFromRobot = noteFieldPose.relativeTo(robotPoseSupplier.get()).getTranslation();
        } else {
            // if (inputs.pipelineIndex != 1) {
            //     setPipelineIndex(1);
            // }

            if (noteInVision()) {

                Translation2d noteTranslationFromCamera = new Translation2d(
                    inputs.noteCalcY,
                    -inputs.noteCalcX
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

    public boolean noteInVision() {
        return (FieldConstants.IS_SIMULATION && ((int) Robot.currentTimestamp/3) % 2 == 0) ||
            (inputs.validResult
            && inputs.targetTxs.length > 0 
            && inputs.limelightTA > 0.5 
            && inputs.targetTxs[0] != 0 
            && inputs.targetTys[0] != 0);
    }
    
    public Pose2d getPose2d() {
        return inputs.botPose2d;
    }

    public double getTagID() {
        return inputs.targetIDs[0];
    }

    public double getLatencyDiffSeconds() {
        return (inputs.latencyPipeline/1000d) - (inputs.latencyCapture/1000d); 
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
        visibleTags = poses.toArray(new Pose3d[0]);
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

    int tagsViable = 0;

    public boolean hasTarget() {

        Pose2d estimatedRobotPose = inputs.botPose2d;

        double singleTagAmbiguityThreshold = Robot.gameMode == GameMode.AUTONOMOUS ? 0.175 : 0.141;
        if (!inputs.validResult
            // || (inputs.limelightTA < singleTagAmbiguityThreshold && inputs.targetIDs.length == 1)
            || (inputs.targetIDs.length > 1 && inputs.limelightTA < CameraConstants.LIMELIGHT_3G_TA_CUTOFF)
            || (estimatedRobotPose.getX() == 0 && estimatedRobotPose.getY() == 0)
            || Double.isNaN(estimatedRobotPose.getX()) 
            || Double.isNaN(estimatedRobotPose.getY()) 
            || Double.isNaN(estimatedRobotPose.getRotation().getRadians())
            || inputs.targetIDs.length == 0
            || Double.valueOf(inputs.targetTxs[0]).equals(null)
            || Double.valueOf(inputs.targetTys[0]).equals(null) 
            || !Double.isFinite(inputs.targetTxs[0])
            || !Double.isFinite(inputs.targetTys[0]))
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
        inputs.currentTime = timingTestEntry.getLastChange();

        // Calculate limelights last update
        timeDifference = inputs.currentTime - inputs.lastUpdate;
        isConnected = timeDifference < CameraConstants.LIMELIGHT_MAX_UPDATE_TIME;

        return isConnected;
    }

    public Pose2d getRobotPoseTargetSpace() {
        return inputs.botPose2dTargetSpace;
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

    public Command setLEDState(boolean enabled) {
        return setLEDState(() -> enabled);
    }

    public void enableLEDS() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    public void disableLEDS() {
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        Results results = getResults();

        inputs.validResult = results != null && results.valid;

        LimelightTarget_Fiducial[] targetFiducials = results.targets_Fiducials;
        // LimelightTarget_Detector[] targetDetectors = results.targets_Detector;

        inputs.targetTxs = new double[targetFiducials.length];
        inputs.targetTys = new double[targetFiducials.length];
        inputs.targetIDs = new int[results.targets_Fiducials.length];
        
        int index = 0;
        
        // if (inputs.pipelineIndex == 0) {
        for (LimelightTarget_Fiducial target : targetFiducials) {
            inputs.targetIDs[index] = (int) target.fiducialID;
            inputs.targetTxs[index] = target.tx;
            inputs.targetTys[index] = target.ty;
            index++;
        }
        inputs.botPose2d = results.getBotPose2d_wpiBlue();
        inputs.botPose3d = results.getBotPose3d_wpiBlue();
        inputs.botPose2dTargetSpace = LimelightHelpers.getBotPose3d_TargetSpace(limelightName).toPose2d();
        // } else if (inputs.pipelineIndex == 1) {
        //     for (LimelightTarget_Detector target : targetDetectors) {
        //         inputs.targetTxs[index] = target.tx;
        //         inputs.targetTys[index] = target.ty;
        //         index++;
        //     }
        //     if (targetDetectors.length > 0) {
        //         targetDetectors[0].calculateXDistance(CameraConstants.LL2Pose.getRotation().toRotation2d().getRadians());
        //         targetDetectors[0].calculateYDistance(CameraConstants.LL2Pose.getZ(), CameraConstants.LL2Pose.getRotation().getY());
        //         inputs.noteCalcX = targetDetectors[0].calcX;
        //         inputs.noteCalcY = targetDetectors[0].calcY;
        //     }
        // }
        inputs.latencyPipeline = results.latency_pipeline;
        inputs.latencyCapture = results.latency_capture;
        inputs.limelightTA = LimelightHelpers.getTA(limelightName);
        inputs.lastUpdate = LimelightHelpers.getLimelightNTTableEntry(limelightName, "tl").getLastChange();
    
    }

}