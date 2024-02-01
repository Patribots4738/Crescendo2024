package frc.robot.subsystems.limelight;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.limelight.LimelightHelpers.Results;
import monologue.Logged;
import monologue.Annotations.Log;

// https://github.com/NAHSRobotics-Team5667/2020-FRC/blob/master/src/main/java/frc/robot/utils/LimeLight.java
public class Limelight extends SubsystemBase implements Logged{

    String limelightName = "limelight";

    @Log.NT
    Pose3d[] poseFiducials;

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
            knownFiducials.add(target.getRobotPose_FieldSpace());
        }

        poseFiducials = knownFiducials.toArray(new Pose3d[0]);
    }

    public Pose2d getPose2d() {
        return LimelightHelpers.getBotPose2d(limelightName);
    }

    public double getTagID() {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public double getLatencyDiffSeconds() {
        return (LimelightHelpers.getLatency_Pipeline(limelightName)/1000d) - (LimelightHelpers.getLatency_Capture(limelightName)/1000d); 
    }

    @Override
    public void periodic() {
        getTags();
    }
}