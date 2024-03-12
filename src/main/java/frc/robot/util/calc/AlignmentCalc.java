package frc.robot.util.calc;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

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
     * Normalize and add two speeds together.
     * 
     * @param controllerSpeeds the controller speeds [-1 , 1]
     * @param autoSpeeds       the auto alignment speeds [-DriveConstants.MAX_SPEED_METERS_PER_SECOND, 
     *                                                     DriveConstants.MAX_SPEED_METERS_PER_SECOND]
     * 
     * @return the normalized ChassisSpeeds with the alignment deadband applied
     */
    public ChassisSpeeds normalizeChassisSpeeds(ChassisSpeeds controllerSpeeds, ChassisSpeeds autoSpeeds) {
        return new ChassisSpeeds(
            MathUtil.applyDeadband(normalizeTwoSpeeds(controllerSpeeds.vyMetersPerSecond, autoSpeeds.vyMetersPerSecond), AutoConstants.AUTO_ALIGNMENT_DEADBAND),
            MathUtil.applyDeadband(normalizeTwoSpeeds(controllerSpeeds.vxMetersPerSecond, autoSpeeds.vxMetersPerSecond), AutoConstants.AUTO_ALIGNMENT_DEADBAND),
            autoSpeeds.omegaRadiansPerSecond / DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND
        );
    }

    /**
     * Normalize and add two speeds together.
     * 
     * @param controllerInput the controller input [0 , 1]
     * @param autoInput       the auto input [0 ,
     *                        DriveConstants.MAX_SPEED_METERS_PER_SECOND]
     * 
     * @return the normalized, combined speed
     */
    public double normalizeTwoSpeeds(double controllerInput, double autoInput) {
        autoInput /= DriveConstants.MAX_SPEED_METERS_PER_SECOND;

        return MathUtil.clamp(
            controllerInput + autoInput,
            -1,
            1);
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
     * Supplier for the amp alignment chassis speeds
     * 
     * @return the supplier for amp alignment chassis speeds
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
     * Supplier for the rotational speeds to align the robot rotationally to the chain.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the supplier for the rotational speeds to align the robot to the chain
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
     * @return        the rotational speeds to align the robot to the source
     */
    public ChassisSpeeds getSourceRotationalSpeeds(double driverX, double driverY) {
        Pose2d source = FieldConstants.GET_SOURCE_POSITION();
        return getRotationalSpeeds(driverX, driverY, source.getRotation());
    }

    /**
     * Supplier for the speeds to align the robot rotationally to the source.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the supplier for the rotational speeds to align the robot to the source
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
     * @return            the rotational speeds to align the robot to the speaker
     */
    public ChassisSpeeds getSpeakerRotationalSpeeds(double driverX, double driverY, ShooterCmds shooterCmds) {
        return 
            getRotationalSpeeds(driverX, driverY, shooterCmds.shooterCalc.calculateRobotAngleToSpeaker(swerve.getPose(), swerve.getRobotRelativeVelocity()));
    }

    public ChassisSpeeds getPassRotationalSpeeds(double driverX, double driverY, ShooterCmds shooterCmds) {
        return 
            getRotationalSpeeds(driverX, driverY, shooterCmds.shooterCalc.calculateRobotAngleToPass(swerve.getPose(), swerve.getRobotRelativeVelocity()));
    }

    /**
     * Calculates the rotational speeds to align the robot to the speaker.
     * 
     * @param driverX      the driver's x input
     * @param driverY      the driver's y input
     * @param desiredAngle the desired angle
     * @return             the rotational speeds to rotationally align the robot
     */
    public ChassisSpeeds getRotationalSpeeds(double driverX, double driverY, Rotation2d desiredAngle) {
        return new ChassisSpeeds(
            driverY * (Robot.isRedAlliance() ? -1 : 1),
            driverX * (Robot.isRedAlliance() ? -1 : 1),
            getAlignmentSpeeds(desiredAngle) / DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND);
    }

    /**
     * Supplier for the speeds to align the robot rotationally to the speaker.
     * 
     * @param driverX     the driver's x input
     * @param driverY     the driver's y input
     * @param shooterCmds the shooter commands
     * @return            the suppoer for the speeds to align the robot to the speaker
     */
    public Supplier<ChassisSpeeds> getSpeakerRotationalSpeedsSupplier(DoubleSupplier driverX, DoubleSupplier driverY, ShooterCmds shooterCmds) {
        return () -> getSpeakerRotationalSpeeds(driverX.getAsDouble(), driverY.getAsDouble(), shooterCmds);
    }

    public Supplier<ChassisSpeeds> getPassRotationalSpeedsSupplier(DoubleSupplier driverX, DoubleSupplier driverY, ShooterCmds shooterCmds) {
        return () -> getPassRotationalSpeeds(driverX.getAsDouble(), driverY.getAsDouble(), shooterCmds);
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

    public Supplier<ChassisSpeeds> getTrapAlignmentSpeedsSupplier() {
        return () -> getTrapAlignmentSpeeds();
    }

    public ChassisSpeeds getTrapControllerSpeeds(double driverX, double driverY) {

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
            ? -driverY
            : closestChain.getTranslation().getY() > FieldConstants.FIELD_HEIGHT_METERS/2.0
                ? driverX * (Robot.isRedAlliance() ? -1 : 1) 
                : -driverX * (Robot.isRedAlliance() ? -1 : 1);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            -x * swerve.getPose().getRotation().getCos(),
            -x * swerve.getPose().getRotation().getSin(),
            0,
            swerve.getPose().getRotation()
        );
    }

    public Supplier<ChassisSpeeds> getTrapControllerSpeedsSupplier(DoubleSupplier driverX, DoubleSupplier driverY) {
        return () -> getTrapControllerSpeeds(driverX.getAsDouble(), driverY.getAsDouble());
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
