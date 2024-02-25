package frc.robot.util.calc;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.commands.subsytemHelpers.ShooterCmds;
import frc.robot.subsystems.Swerve;
import frc.robot.util.constants.Constants.AutoConstants;
import frc.robot.util.constants.Constants.FieldConstants;

public class AlignmentCalc {
    

    private Swerve swerve;

    public AlignmentCalc(Swerve swerve) {
        this.swerve = swerve;
    }

    /**
     * Calculates the rotational HDC speeds to align the robot to the desired angle.
     * 
     * @param desiredAngle the desired angle
     * @return             the rotational HDC speeds to align the robot to the desired angle
     */
    public double getAlignmentSpeeds(Rotation2d desiredAngle) {
        return MathUtil.applyDeadband(AutoConstants.HDC.getThetaController().calculate(
            swerve.getPose().getRotation().getRadians(),
            desiredAngle.getRadians()),  0.02);
    }

    /**
     * Calculates the rotational speeds to align the robot to the amp.
     * 
     * @return the rotational speeds to align the robot to the amp
     */
    public ChassisSpeeds getAmpAlignmentSpeeds() {
        Pose2d ampPose = FieldConstants.GET_AMP_POSITION();
        Pose2d desiredPose = new Pose2d(
            ampPose.getX(),
            swerve.getPose().getY(),
            ampPose.getRotation()
        );
        swerve.setDesiredPose(desiredPose);
        return
            AutoConstants.HDC.calculate(
                swerve.getPose(),
                desiredPose,
                0,
                desiredPose.getRotation()
            );
    }

    /**
     * Supplier for the Command to align the robot to the amp.
     * 
     * @return the command to align the robot to the amp
     */
    public Supplier<ChassisSpeeds> getAmpAlignmentSpeedsSupplier() {
        return () -> getAmpAlignmentSpeeds();
    }

    /**
     * Calculates the rotational speeds to align the robot to the chain.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the rotational speeds to align the robot to the chain
     */
    public ChassisSpeeds getChainRotationalSpeeds(double driverX, double driverY) {
        Pose2d closestChain = PoseCalculations.getClosestChain(swerve.getPose());
        return new ChassisSpeeds(
            driverY * (Robot.isRedAlliance() ? -1 : 1),
            driverX * (Robot.isRedAlliance() ? -1 : 1),
            getAlignmentSpeeds(closestChain.getRotation())
        );
    }

    /**
     * Supplier for the Command to align the robot rotationally to the chain.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the command to align the robot to the chain
     */
    public Supplier<ChassisSpeeds> getChainRotationalSpeedsSupplier(double driverX, double driverY) {
        return () -> getChainRotationalSpeeds(driverX, driverY);
    }

    /**
     * Calculates the rotational speeds to align the robot to the source of the field.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return     the rotational speeds to align the robot to the source
     */
    public ChassisSpeeds getSourceRotationalSpeeds(double driverX, double driverY) {
        Pose2d source = FieldConstants.GET_SOURCE_POSITION();
        return new ChassisSpeeds(
            driverY * (Robot.isRedAlliance() ? -1 : 1),
            driverX * (Robot.isRedAlliance() ? -1 : 1),
            getAlignmentSpeeds(source.getRotation())
        );
    }

    /**
     * Supplier for the Command to align the robot rotationally to the source of the field.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the command to align the robot to the source
     */
    public Supplier<ChassisSpeeds> getSourceRotationalSpeedsSupplier(double driverX, double driverY) {
        return () -> getSourceRotationalSpeeds(driverX, driverY);
    }

    /**
     * Calculates the rotational speeds to align the robot to the speaker.
     * 
     * @param driverX   the driver's x input
     * @param driverY   the driver's y input
     * @param shooterCmds the shooter commands
     * @return        the rotational speeds to align the robot to the speaker
     */
    public ChassisSpeeds getSpeakerRotationalSpeeds(double driverX, double driverY, ShooterCmds shooterCmds) {
        return new ChassisSpeeds(
            driverY * (Robot.isRedAlliance() ? -1 : 1),
            driverX * (Robot.isRedAlliance() ? -1 : 1),
            getAlignmentSpeeds(shooterCmds.shooterCalc.calculateSWDRobotAngleToSpeaker(swerve.getPose(), swerve.getFieldRelativeVelocity())));
    }

    /**
     * Supplier for the Command to align the robot rotationally to the speaker.
     * 
     * @param driverX   the driver's x input
     * @param driverY   the driver's y input
     * @param shooterCmds the shooter commands
     * @return          the command to align the robot to the speaker
     */
    public Supplier<ChassisSpeeds> getSpeakerRotationalSpeedsSupplier(double driverX, double driverY, ShooterCmds shooterCmds) {
        return () -> getSpeakerRotationalSpeeds(driverX, driverY, shooterCmds);
    }

    /**
     * Calculates the rotational speeds to align the robot to the trap.
     * 
     * @return the rotational speeds to align the robot to the trap
     */
    public ChassisSpeeds getTrapAlignmentSpeeds() {
        Pose2d closestTrap = PoseCalculations.getClosestChain(swerve.getPose());
        Pose2d stage = FieldConstants.GET_STAGE_POSITION();
        double distance = swerve.getPose().relativeTo(stage).getTranslation().getNorm();
        double x = stage.getX() + distance * closestTrap.getRotation().getCos();
        double y = stage.getY() + distance * closestTrap.getRotation().getSin();
        Pose2d desiredPose = new Pose2d(
            x,
            y,
            closestTrap.getRotation()
        );
        swerve.setDesiredPose(desiredPose);
        return
            AutoConstants.HDC.calculate(
                swerve.getPose(),
                desiredPose,
                0,
                desiredPose.getRotation()
            );
    }

    /**
     * Supplier for the Command to align the robot to the trap.
     * 
     * @return the command to align the robot to the trap
     */
    public Supplier<ChassisSpeeds> getTrapAlignmentSpeedsSupplier() {
        return () -> getTrapAlignmentSpeeds();
    }

    /**
     * Detects if the robot is on the opposite side of the field.
     * Uses the robot's x position to determine if it has crossed the centerline.
     * 
     * @return true if the robot is on the opposite side of the field
     */
    public boolean onOppositeSide() {
        return Robot.isRedAlliance() 
            ? swerve.getPose().getX() < FieldConstants.CENTERLINE_X 
            : swerve.getPose().getX() > FieldConstants.CENTERLINE_X;
    }
}
