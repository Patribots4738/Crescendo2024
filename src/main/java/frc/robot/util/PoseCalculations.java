package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.FieldConstants.ChainPosition;
import monologue.Annotations.Log;

public class PoseCalculations {
    
    @Log.NT
    public static ChainPosition getChainPosition(Pose2d position) {
        Pose2d closestChainPose;
        
        double minDifference = 10000;
        int closestChainIndex = 0;

        for (int i = 0; i < FieldConstants.CHAIN_POSITIONS.length; i++) {
            Pose2d currentChainPose = FieldConstants.CHAIN_POSITIONS[i];
            double currentChainDistance = position.getTranslation().getDistance(currentChainPose.getTranslation());

            minDifference = currentChainDistance < minDifference 
                ? currentChainDistance
                : minDifference;

            closestChainIndex = currentChainDistance < minDifference 
                ? i
                : closestChainIndex;
        }

        closestChainPose = FieldConstants.CHAIN_POSITIONS[closestChainIndex];

        Pose2d relativePosition = position.relativeTo(closestChainPose);

        if (relativePosition.getX() < Units.metersToInches(FieldConstants.CHAIN_LENGTH_METERS/3)) {
            return ChainPosition.LEFT;
        } else if (relativePosition.getX() > Units.metersToInches(FieldConstants.CHAIN_LENGTH_METERS/3)) {
            return ChainPosition.RIGHT;
        }
        return ChainPosition.CENTER;
    }

}