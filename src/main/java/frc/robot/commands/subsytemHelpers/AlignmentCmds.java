package frc.robot.commands.subsytemHelpers;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Swerve;
import frc.robot.util.calc.AlignmentCalc;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.constants.Constants.FieldConstants;
import frc.robot.util.testing.PatritionalCommand;

public class AlignmentCmds {
    
    private Climb climb;
    private Swerve swerve;
    private ShooterCmds shooterCmds;

    public AlignmentCalc alignmentCalc;

    public AlignmentCmds(Swerve swerve, Climb climb, ShooterCmds shooterCmds) {
        this.climb = climb;
        this.swerve = swerve;
        this.shooterCmds = shooterCmds;
        this.alignmentCalc = new AlignmentCalc(swerve);
    }

    public Command getAutoAlignmentCommand(Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds) {
        return 
            swerve.getDriveCommand(() -> {
                ChassisSpeeds controllerSpeedsGet = controllerSpeeds.get();
                ChassisSpeeds autoSpeedsGet = autoSpeeds.get();
                return new ChassisSpeeds(
                        MathUtil.applyDeadband((controllerSpeedsGet.vxMetersPerSecond + autoSpeedsGet.vxMetersPerSecond), 0.1),
                        MathUtil.applyDeadband(-(controllerSpeedsGet.vyMetersPerSecond + autoSpeedsGet.vyMetersPerSecond), 0.1),
                        controllerSpeedsGet.omegaRadiansPerSecond + autoSpeedsGet.omegaRadiansPerSecond);
            }, () -> false);
    }

    public Command ampAlignmentCommand(DoubleSupplier driverX) {
        return 
            getAutoAlignmentCommand(
                alignmentCalc.getAmpAlignmentSpeedsSupplier(), 
                () -> 
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        driverX.getAsDouble() * (Robot.isRedAlliance() ? 1 : -1),
                        0,
                        swerve.getPose().getRotation()
                    )
            );
    }

    public Command chainRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getChainRotationalSpeedsSupplier(driverX, driverY), 
                () -> true
        );
    }    

    /**
     * Command to align the robot rotationally to the source of the field.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the command to align the robot to the source
     */
    public Command sourceRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getSourceRotationalSpeedsSupplier(driverX, driverY), 
                () -> true
        );
    }

    public Command speakerRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY, ShooterCmds shooterCmds) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getSpeakerRotationalSpeedsSupplier(
                    driverX,
                    driverY,
                    shooterCmds),
                () -> true);
    }

    /**
     * Command to align the robot to the trap of the field.
     * multiplies the driverY by the cosine and sine to get the x and y components of the field relative speed
     * '
     * @param driverY the driver's y input
     * @return        the command to align the robot to the trap
     */
    public Command trapAlignmentCommand(DoubleSupplier driverX, DoubleSupplier driverY) {
        return 
            getAutoAlignmentCommand(
                alignmentCalc::getTrapAlignmentSpeeds, 
                () -> getTrapAlignmentSpeeds(driverX, driverY)
            );
    }

    public ChassisSpeeds getTrapAlignmentSpeeds(DoubleSupplier driverX, DoubleSupplier driverY) {

        // Get the current chain that we are aligning to and the current robot pose
        Pose2d closestChain = PoseCalculations.getClosestChain(swerve.getPose());

        // Make the pose always relative to the blue alliance
        // Closest chain is the actual chain position
        // when we use our comparitors below, we use things from
        // the origin of the field's persepctive
        if (Robot.isRedAlliance()) {
            closestChain = GeometryUtil.flipFieldPose(closestChain);
        }

        // If we are on the left chain, then use the positive driver X axis
        // If we are on the right chain, then use the negative driver X axis
        // If we are on the back chain, then use the positive driver Y axis
        double x = (closestChain.getTranslation().getX() == FieldConstants.CHAIN_POSITIONS.get(1).getX()) 
            ? -driverY.getAsDouble() 
            : closestChain.getTranslation().getY() > FieldConstants.FIELD_HEIGHT_METERS/2.0
                ? driverX.getAsDouble() * (Robot.isRedAlliance() ? -1 : 1) 
                : -driverX.getAsDouble() * (Robot.isRedAlliance() ? -1 : 1);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            -x * swerve.getPose().getRotation().getCos(),
            -x * swerve.getPose().getRotation().getSin(),
            0,
            swerve.getPose().getRotation()
        );
    }

    /**
     * Command to align the robot to the wing of the field.
     * multiplies the driverY by the cosine and sine to get the x and y components of the field relative speed
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the command to align the robot to the wing
     */
    public Command wingRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return
            new PatritionalCommand(
                chainRotationalAlignment(driverX, driverY),
                speakerRotationalAlignment(driverX, driverY, shooterCmds),
                climb::getHooksUp);
    }
}
