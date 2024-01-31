package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.FieldConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class PoseCalculations implements Logged {

    @Log.NT
    private Pose3d chainLeft = new Pose3d();
    @Log.NT
    private Pose3d chainRight = new Pose3d();

    public Pair<Double, Double> getChainPosition(Pose2d position) {
        Pose2d closestChainPose;

        double minDifference = 10000;
        int closestChainIndex = 0;

        int startingIndex = FieldConstants.ALLIANCE.equals(Optional.of(Alliance.Red)) ? 3 : 0;
        int endingIndex = FieldConstants.ALLIANCE.equals(Optional.of(Alliance.Red)) ? 6 : 3;

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
            chainLeft = new Pose3d();
            chainRight = new Pose3d();
            return Pair.of(0d, 0d);
        }

        double leftIntercept = getChainIntercept(relativePosition.getY() + ClimbConstants.DISTANCE_FROM_ORIGIN_METERS);
        double rightIntercept = getChainIntercept(relativePosition.getY() - ClimbConstants.DISTANCE_FROM_ORIGIN_METERS);

        chainLeft = new Pose3d(
                        0,
                        0,
                        leftIntercept - 0.6, // Convert from the chain's height to the climb's height w/ some extra space
                        new Rotation3d(
                            0,
                            0,
                            0));

        chainRight = new Pose3d(
                        0,
                        0,
                        rightIntercept - 0.6, // Convert from the chain's height to the climb's height w/ some extra space
                        new Rotation3d(
                            0,
                            0,
                            0));

        return Pair.of(
            getChainIntercept(relativePosition.getY() + ClimbConstants.DISTANCE_FROM_ORIGIN_METERS), 
            getChainIntercept(relativePosition.getY() - ClimbConstants.DISTANCE_FROM_ORIGIN_METERS));
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
    public double getChainIntercept(double x) {
        // The ds here turn the integers into doubles
        // so that integer divion does not occur.
        double calculation = 3d / 10d * Math.pow(x, 2) + 0.725;
        // Clamp the output to be no lower than the lowest point of the chain,
        // and no higher than the extension limit of our elevator
        return MathUtil.clamp(calculation, 0.725, ClimbConstants.EXTENSION_LIMIT_METERS);
    }

}