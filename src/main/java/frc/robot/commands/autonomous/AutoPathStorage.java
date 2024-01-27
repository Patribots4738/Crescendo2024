package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.FieldConstants;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * This file represents all of the auto paths that we will have
 * They will be primarily comiled through
 * PathPlannerTrajectory.importChoreoPaths,
 * with each segment having its own method to make sure that the modularity
 * stays clean
 */
public class AutoPathStorage {

    // This suplier is used to determine whether or not the robot has a piece
    // Could be a sensor, motor current, or something else
    private final BooleanSupplier hasPieceSupplier;
    private final Map<String, PathPlannerPath> pathCache = new HashMap<>();

    public AutoPathStorage(BooleanSupplier hasPieceSupplier) {
        this.hasPieceSupplier = hasPieceSupplier;
        preloadPaths();
    }

    private void preloadPaths() {
        for (int i = 1; i <= FieldConstants.CENTER_NOTE_COUNT; i++) {

            int shootingIndex = (i * 2);
            
            String shootingDownPathName1 = AutoConstants.SHOOTING_DOWN_PATH_NAME + (shootingIndex-1);
            String shootingUpPathName1   = AutoConstants.SHOOTING_UP_PATH_NAME   + (shootingIndex-1);
            String shootingDownPathName2 = AutoConstants.SHOOTING_DOWN_PATH_NAME + (shootingIndex);
            String shootingUpPathName2   = AutoConstants.SHOOTING_UP_PATH_NAME   + (shootingIndex);
            // Skipping index = i
            String skippingDownPathName  = AutoConstants.SKIPPING_DOWN_PATH_NAME + i;
            String skippingUpPathName    = AutoConstants.SKIPPING_UP_PATH_NAME   + i;

            pathCache.put(shootingDownPathName1, PathPlannerPath.fromChoreoTrajectory(shootingDownPathName1));
            pathCache.put(shootingUpPathName1  , PathPlannerPath.fromChoreoTrajectory(shootingUpPathName1));
            
            // On the last index, 
            // we only have the first shooting path
            if (shootingIndex == FieldConstants.CENTER_NOTE_COUNT * 2) {
                return;
            }

            pathCache.put(shootingDownPathName2, PathPlannerPath.fromChoreoTrajectory(shootingDownPathName2));
            pathCache.put(shootingUpPathName2  , PathPlannerPath.fromChoreoTrajectory(shootingUpPathName2));
            pathCache.put(skippingDownPathName , PathPlannerPath.fromChoreoTrajectory(skippingDownPathName));
            pathCache.put(skippingUpPathName   , PathPlannerPath.fromChoreoTrajectory(skippingUpPathName));
            
        }
    }

    /**
     * Generates a command to follow a specific path in autonomous mode.
     * 
     * @param startingIndex The starting index of the path.
     * @param noteCount     The number of notes to get.
     * @param goingUp       Indicates whether the robot is going up or down the
     *                      path.
     * @return The generated command to follow the full path.
     */
    public Command generateCenterLineComplete(int startingIndex, int noteCount, boolean goingUp) {
        SequentialCommandGroup fullPath = new SequentialCommandGroup();

        String shootingPathName;
        String skippingPathName;

        if (goingUp) {
            startingIndex = Math.abs(startingIndex-6);
            shootingPathName = AutoConstants.SHOOTING_UP_PATH_NAME;
            skippingPathName = AutoConstants.SKIPPING_UP_PATH_NAME;
        } else /* we're going down */ {
            shootingPathName = AutoConstants.SHOOTING_DOWN_PATH_NAME;
            skippingPathName = AutoConstants.SKIPPING_DOWN_PATH_NAME;
        }
        
        startingIndex = MathUtil.clamp(startingIndex, 1, 5);
        noteCount = MathUtil.clamp(noteCount, 1, 5);

        for (int i = startingIndex; i <= noteCount; i++) {

            int shootingIndex = (i * 2);

            PathPlannerPath shootPiece = getPathFromCache(shootingPathName + (shootingIndex - 1));
            
            if (shootingIndex == noteCount * 2) {
                fullPath.addCommands(AutoBuilder.followPath(shootPiece));
                return fullPath;
            }

            PathPlannerPath skipShot = getPathFromCache(skippingPathName + i);
            PathPlannerPath getNextPiece = getPathFromCache(shootingPathName + (shootingIndex));

            fullPath.addCommands(
                    Commands.either(
                            AutoBuilder.followPath(shootPiece).andThen(AutoBuilder.followPath(getNextPiece)),
                            AutoBuilder.followPath(skipShot),
                            hasPieceSupplier));
        }
        return fullPath;
    }

    private PathPlannerPath getPathFromCache(String pathName) {
        return pathCache.get(pathName);
    }
}
