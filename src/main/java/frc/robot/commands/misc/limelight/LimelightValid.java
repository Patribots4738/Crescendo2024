package frc.robot.commands.misc.limelight;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.misc.limelight.Limelight;
import frc.robot.util.calc.LimelightHelpers;

public class LimelightValid extends Command {
    
    private final Limelight limelight;
    private Swerve swerve;
    private DoubleSupplier leftAxis;

    public LimelightValid(Limelight limelight, Swerve swerve, DoubleSupplier leftAxis) {
        this.limelight = limelight;
        this.swerve = swerve;
        this.leftAxis = leftAxis;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        runLimelightCode(leftAxis);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void runLimelightCode(DoubleSupplier leftAxis) {
        // Create an "Optional" object that contains the estimated pose of the robot
        // This can be present (sees tag) or not present (does not see tag)
        LimelightHelpers.Results result = limelight.getResults();
        // The skew of the tag represents how confident the camera is
        // If the result of the estimatedRobotPose exists,
        // and the skew of the tag is less than 3 degrees,
        // then we can confirm that the estimated position is realistic
        if ( // check validity
            ((leftAxis.getAsDouble() > 0 && !(result.botpose[0] == 0 && result.botpose[1] == 0) )
            // check if good tag
            && (LimelightHelpers.getTA("limelight") >= 0.3 
                || result.targets_Fiducials.length > 1 && LimelightHelpers.getTA("limelight") > 0.4))
            && limelight.getRobotPoseTargetSpace().getTranslation().getNorm() < 3.25
        ) {
            Pose2d estimatedRobotPose = result.getBotPose2d_wpiBlue();
            if (Double.isNaN(estimatedRobotPose.getX()) 
                || Double.isNaN(estimatedRobotPose.getY()) 
                || Double.isNaN(estimatedRobotPose.getRotation().getRadians())) {
                return;
            }
            swerve.getPoseEstimator().addVisionMeasurement( 
                estimatedRobotPose,
                Robot.currentTimestamp - limelight.getLatencyDiffSeconds());
        }
    }
}