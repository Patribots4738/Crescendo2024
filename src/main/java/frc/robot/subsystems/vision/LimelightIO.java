package frc.robot.subsystems.vision;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface LimelightIO {

    @AutoLog
    class LimelightIOInputs {
        
        public boolean validResult = true;
        public int[] targetIDs = new int[0]; 
        public double[] targetTxs =  new double[0];
        public double[] targetTys =  new double[0];
        public double limelightTA = 0.0;
        public Pose2d botPose2d = new Pose2d();
        public Pose3d botPose3d = new Pose3d();
        public Pose2d botPose2dTargetSpace = new Pose2d();
        public double fiducialID = 0.0;
        public double latencyPipeline = 0.0;
        public double latencyCapture = 0.0;
        public double noteCalcY = 0.0;
        public double noteCalcX = 0.0;
        public long currentTime = 0;
        public long lastUpdate = 0;
        public int pipelineIndex = 0;

    }

    default void updateInputs(LimelightIOInputs inputs) {}
    
}
