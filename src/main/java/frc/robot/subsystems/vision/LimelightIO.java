package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.calc.LimelightHelpers.Results;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

public interface LimelightIO {

    @AutoLog
    class LimelightIOInputs {

        public Results results = new Results();
        public double limelightTA = 0.0;
        public Pose2d botPose2d = new Pose2d();
        public Pose3d botPose3d = new Pose3d();
        public double fiducialID = 0.0;

    }

    default void updateInputs(LimelightIOInputs inputs) {}
    
}
