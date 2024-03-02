package frc.robot.util.calc;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.commands.managers.ShooterCmds;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;

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
        return getRotationalSpeeds(driverX, driverY, closestChain.getRotation());
    }

    /**
     * Supplier for the Command to align the robot rotationally to the chain.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the command to align the robot to the chain
     */
    public Supplier<ChassisSpeeds> getChainRotationalSpeedsSupplier(DoubleSupplier driverX, DoubleSupplier driverY) {
        return () -> getChainRotationalSpeeds(driverX.getAsDouble(), driverY.getAsDouble());
    }

    /**
     * Calculates the rotational speeds to align the robot to the source of the
     * field.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return the rotational speeds to align the robot to the source
     */
    public ChassisSpeeds getSourceRotationalSpeeds(double driverX, double driverY) {
        Pose2d source = FieldConstants.GET_SOURCE_POSITION();
        return getRotationalSpeeds(driverX, driverY, source.getRotation());
    }

    /**
     * Supplier for the Command to align the robot rotationally to the source of the field.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the command to align the robot to the source
     */
    public Supplier<ChassisSpeeds> getSourceRotationalSpeedsSupplier(DoubleSupplier driverX, DoubleSupplier driverY) {
        return () -> getSourceRotationalSpeeds(driverX.getAsDouble(), driverY.getAsDouble());
    }

    /**
     * Calculates the rotational speeds to align the robot to the speaker.
     * 
     * @param driverX     the driver's x input
     * @param driverY     the driver's y input
     * @param shooterCmds the shooter commands
     * @return the rotational speeds to align the robot to the speaker
     */
    public ChassisSpeeds getSpeakerRotationalSpeeds(double driverX, double driverY, ShooterCmds shooterCmds) {
        return 
            getRotationalSpeeds(driverX, driverY, shooterCmds.shooterCalc.calculateRobotAngleToSpeaker(swerve.getPose(), swerve.getRobotRelativeVelocity()));
    }

    /**
     * Calculates the rotational speeds to align the robot to the speaker.
     * 
     * @param driverX      the driver's x input
     * @param driverY      the driver's y input
     * @param desiredAngle the desired angle
     * @return the rotational speeds to align the robot
     */
    public ChassisSpeeds getRotationalSpeeds(double driverX, double driverY, Rotation2d desiredAngle) {
        return new ChassisSpeeds(
            driverY * (Robot.isRedAlliance() ? -1 : 1),
            driverX * (Robot.isRedAlliance() ? -1 : 1),
            getAlignmentSpeeds(desiredAngle) / DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND);
    }

    /**
     * Supplier for the Command to align the robot rotationally to the speaker.
     * 
     * @param driverX   the driver's x input
     * @param driverY   the driver's y input
     * @param shooterCmds the shooter commands
     * @return          the command to align the robot to the speaker
     */
    public Supplier<ChassisSpeeds> getSpeakerRotationalSpeedsSupplier(DoubleSupplier driverX, DoubleSupplier driverY, ShooterCmds shooterCmds) {
        return () -> getSpeakerRotationalSpeeds(driverX.getAsDouble(), driverY.getAsDouble(), shooterCmds);
    }

    /**
     * Calculates the rotational speeds to align the robot to the trap.
     * 
     * @return the rotational speeds to align the robot to the trap
     */
    public ChassisSpeeds getTrapAlignmentSpeeds() {
        Pose2d currentRobotPose = swerve.getPose();
        Pose2d closestTrap = PoseCalculations.getClosestChain(currentRobotPose);
        Pose2d stage = FieldConstants.GET_STAGE_POSITION();
        double distance = currentRobotPose.relativeTo(stage).getTranslation().getNorm();
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
     * Detects if the robot is on the opposite side of the field.
     * Uses the robot's x position to determine if it has crossed the centerline.
     * 
     * @return true if the robot is on the opposite side of the field
     */
    public boolean onOppositeSide() {
        return Robot.isRedAlliance() 
            ? swerve.getPose().getX() < FieldConstants.BLUE_WING_X 
            : swerve.getPose().getX() > FieldConstants.RED_WING_X;
    }
}
