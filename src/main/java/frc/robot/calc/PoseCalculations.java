package frc.robot.calc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;

public class PoseCalculations implements Logged {

    /**
     * Calculates the intercepts of the chain based on the given robot position.
     * 
     * @param position the robot's position
     * @return a Pair containing the left and right intercepts of the chain
     */
    public static Pair<Double, Double> getChainIntercepts(Pose2d position) {
        Pose2d closestChainPose = getClosestChain(position);

        Pose2d relativePosition = position.relativeTo(closestChainPose);

        if (Math.abs(relativePosition.getTranslation().getX()) > 1) {
            return Pair.of(0d, 0d);
        }

        double leftIntercept = getChainIntercept(relativePosition.getY() + ClimbConstants.DISTANCE_FROM_ORIGIN_METERS);
        double rightIntercept = getChainIntercept(relativePosition.getY() - ClimbConstants.DISTANCE_FROM_ORIGIN_METERS);

        return Pair.of(leftIntercept - 0.45, rightIntercept - 0.45);
    }

    /**
     * Calculates the closest chain position to a given pose.
     *
     * @param position The pose for which to find the closest chain position.
     * @return The closest chain position to the given pose.
     */
    public static Pose2d getClosestChain(Pose2d position) {
        return position.nearest(FieldConstants.GET_CHAIN_POSITIONS());
    }

    @Log
    private static Pose2d closestShootingPose;

    /**
     * Given the robot's position, return the best shooting pose. Given the preset shooting positions
     * on the field, defined in the Auto naming guide in tldraw
     * @param position
     * @return
     */
    @Log
    public static Pose2d getBestShootingPose(Pose2d position) {
        double yValue = position.getY();
        if (yValue > 5)
            // L
            return FieldConstants.GET_SHOOTING_POSITIONS().get(0);
        if (yValue > 1.8)
            // M
            return FieldConstants.GET_SHOOTING_POSITIONS().get(2);
        // R
        return FieldConstants.GET_SHOOTING_POSITIONS().get(1);
    }

    /**
     * Given the robot's position, return the best shooting pose as a string. Given the preset shooting positions
     * on the field, defined in the Auto naming guide in tldraw
     * @param position
     * @return
     */
    public static String getBestShootingPoseString(Pose2d position) {
        closestShootingPose = getBestShootingPose(position);
        if (closestShootingPose.equals(FieldConstants.GET_SHOOTING_POSITIONS().get(0)))
            return "L";

        if (closestShootingPose.equals(FieldConstants.GET_SHOOTING_POSITIONS().get(1)))
            return "R";

        if (closestShootingPose.equals(FieldConstants.GET_SHOOTING_POSITIONS().get(2)))
            return "M";
        
        return "if you managed to get this to run you deserve a cookie... wait.. no. i want that cookie.";
    }

    /**
     * Given an intercept of the chain, return the height of the chain at that
     * location.
     * 
     * @param x The position along the X axis of the chain
     *          as seen in https://www.desmos.com/calculator/84ioficbl2
     *          For some reason, that desmos breaks on chrome on my home computer
     *          please send help... i used edge to make it :(
     * 
     * @return The height of the chain at that location
     */
    private static double getChainIntercept(double x) {
        // The ds here turn the integers into doubles
        // so that integer division does not occur.
        double calculation = 3d / 10d * Math.pow(x, 2) + 0.725;
        // Clamp the output to be no lower than the lowest point of the chain,
        // and no higher than the extension limit of our elevator
        return MathUtil.clamp(calculation, 0.725, ClimbConstants.EXTENSION_LIMIT_METERS);
    }

    @Log
    private static Rotation2d currentAngleToSpeaker = new Rotation2d();
    @Log
    public static Pose2d desiredSWDPose = new Pose2d();
    @Log
    private static double degreesToSpeakerReferenced = 0;
    @Log
    private static double angleDifference = 0;

    /**
     * Calculates the angle to the speaker based on the robot's pose and velocity.
     * This is to shoot while driving, but would also work while stationary.
     * 
     * @param robotPose     The current pose of the robot.
     * @param robotVelocity The velocity of the robot.
     * 
     * @return              The angle to the speaker in the form of a Rotation2d object.
     */
    public static Rotation2d calculateRobotAngleToPose(Pose2d robotPose, ChassisSpeeds robotVelocity, Pose2d target, double averageShooterSpeed) {
        Translation2d velocityVectorToSpeaker = getVelocityVectorToSpeaker(robotPose, robotVelocity);
        double velocityTangent = velocityVectorToSpeaker.getX();

        Pose2d poseRelativeToTarget = robotPose.relativeTo(target);
        Rotation2d currentAngleToTarget = new Rotation2d(poseRelativeToTarget.getX(), poseRelativeToTarget.getY());
        double velocityArcTan = Math.atan2(
            velocityTangent,
            ShooterCalc.rpmToVelocity(averageShooterSpeed)
        );
        // Calculate the desired rotation to the speaker, taking into account the tangent velocity
        // Add PI because the speaker opening is the opposite direction that the robot needs to be facing
        Rotation2d desiredRotation2d = Rotation2d.fromRadians(MathUtil.angleModulus(
            currentAngleToTarget.getRadians() + velocityArcTan + Math.PI
        ));

        // Update the robot's pose with the desired rotation
        desiredSWDPose = new Pose2d(robotPose.getTranslation(), desiredRotation2d);

        // Return the desired rotation
        return desiredRotation2d;
    }

    public static Rotation2d calculateRobotAngleToPass(Pose2d robotPose) {
        return calculateRobotAngleToPass(robotPose, new ChassisSpeeds(), 0);
    }

    public static Rotation2d calculateRobotAngleToPass(Pose2d robotPose, ChassisSpeeds robotVelocity, double averageShooterSpeed) {
        return calculateRobotAngleToPose(robotPose, robotVelocity, FieldConstants.GET_PASS_TARGET_POSITION(), averageShooterSpeed);
    }

    public static Rotation2d calculateRobotAngleToSpeaker(Pose2d pose) {
        return calculateRobotAngleToSpeaker(pose, new ChassisSpeeds(), 0);
    }

    public static Rotation2d calculateRobotAngleToSpeaker(Pose2d pose, ChassisSpeeds robotVelocity, double averageShooterSpeed) {
        return calculateRobotAngleToPose(pose, robotVelocity, FieldConstants.GET_SPEAKER_POSITION(), averageShooterSpeed);
    }

    public static Rotation2d calculateRobotAngleToSpeaker(Translation2d translation) {
        return calculateRobotAngleToSpeaker(new Pose2d(translation, new Rotation2d()));
    }

    public static Translation2d getVelocityVectorToSpeaker(Pose2d robotPose, ChassisSpeeds robotVelocity) {
        // Calculate the robot's pose relative to the speaker
        Pose2d poseRelativeToSpeaker = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());

        // Calculate the current angle to the speaker
        currentAngleToSpeaker = new Rotation2d(poseRelativeToSpeaker.getX(), poseRelativeToSpeaker.getY());

        // Convert the robot's velocity to a Rotation2d object
        Rotation2d velocityRotation2d = new Rotation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

        // Calculate the total speed of the robot
        double totalSpeed = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

        double angleDifference = velocityRotation2d.getRadians() - currentAngleToSpeaker.getRadians();
        // Calculate the component of the velocity that is tangent to the speaker
        double velocityTangentToSpeaker = totalSpeed * Math.sin(angleDifference);

        double velocityNormalToSpeaker = totalSpeed * Math.cos(angleDifference);

        if (Robot.isBlueAlliance()) {
            velocityTangentToSpeaker *= -1;
            velocityNormalToSpeaker *= -1;
        }
        
        return new Translation2d(velocityTangentToSpeaker, velocityNormalToSpeaker);
    }

    /**
     * Detects if the robot is on the opposite side of the field.
     * Uses the robot's x position to determine if it has crossed the centerline.
     * 
     * @return true if the robot is on the opposite side of the field
     */
    public static boolean onOppositeSide(Pose2d pose) {
        return Robot.isRedAlliance() 
            ? pose.getX() < FieldConstants.BLUE_WING_X 
            : pose.getX() > FieldConstants.RED_WING_X;
    }

}