package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.FieldConstants;
import monologue.Logged;

public class PoseCalculations implements Logged {

    public static Pair<Double, Double> getChainIntercepts(Pose2d position) {
        Pose2d closestChainPose;

        double minDifference = 10000;
        int closestChainIndex = 0;

        int startingIndex = FieldConstants.IS_RED_ALLIANCE() ? 3 : 0;
        int endingIndex = FieldConstants.IS_RED_ALLIANCE() ? 6 : 3;

        for (int i = startingIndex; i < endingIndex; i++) {
            Pose2d currentChainPose = FieldConstants.CHAIN_POSITIONS[i];
            double currentChainDistance = position.getTranslation().getDistance(currentChainPose.getTranslation());

            if (currentChainDistance < minDifference) {
                minDifference = currentChainDistance;
                closestChainIndex = i;
            }
        }

        closestChainPose = FieldConstants.CHAIN_POSITIONS[closestChainIndex];

        Pose2d relativePosition = position.relativeTo(closestChainPose);

        if (Math.abs(relativePosition.getTranslation().getX()) > 1) {
            return Pair.of(0d, 0d);
        }

        double leftIntercept = getChainIntercept(relativePosition.getY() + ClimbConstants.DISTANCE_FROM_ORIGIN_METERS);
        double rightIntercept = getChainIntercept(relativePosition.getY() - ClimbConstants.DISTANCE_FROM_ORIGIN_METERS);

        return Pair.of(leftIntercept - 0.6, rightIntercept - 0.6);
    }

    /**
     * Given an intercept of the chain, return the height of the chain at that
     * location.
     * 
     * @param x The posision along the X axis of the chain
     *          as seen in https://www.desmos.com/calculator/84ioficbl2
     *          For some reason, that desmos breaks on chrome on my home computer
     *          please send help... i used edge to make it :(
     * 
     * @return The height of the chain at that location
     */
    private static double getChainIntercept(double x) {
        // The ds here turn the integers into doubles
        // so that integer divion does not occur.
        double calculation = 3d / 10d * Math.pow(x, 2) + 0.725;
        // Clamp the output to be no lower than the lowest point of the chain,
        // and no higher than the extension limit of our elevator
        return MathUtil.clamp(calculation, 0.725, ClimbConstants.EXTENSION_LIMIT_METERS);
    }

}