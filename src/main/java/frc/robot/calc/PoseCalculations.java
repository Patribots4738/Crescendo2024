package frc.robot.calc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.FieldConstants;
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

}