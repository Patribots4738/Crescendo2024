package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.limelight.LimelightHelpers.Results;

public interface LimelightIO {

    public Results getResults();
    
    public Pose2d getPose2d();
    
    public double getTagID();

    public double getLatencyDiffSeconds();
}
