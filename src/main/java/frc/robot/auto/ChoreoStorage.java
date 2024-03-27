package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import monologue.Logged;
import monologue.Annotations.Log;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * This file represents all of the auto paths that we will have
 * They will be primarily comiled through
 * PathPlannerTrajectory.importChoreoPaths,
 * with each segment having its own method 
 * to make sure that the modularity stays clean
 */
public class ChoreoStorage implements Logged {

    private final BooleanSupplier hasPieceSupplier;
    private final Map<String, PathPlannerPath> pathCache = new HashMap<>();
    @Log.NT
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * Creates a new AutoPathStorage object.
     * @param hasPieceSupplier A supplier that returns whether or not the robot has a piece.
     *                         This could be a sensor, motor current, or other system.
     */
    public ChoreoStorage(BooleanSupplier hasPieceSupplier) {
        this.hasPieceSupplier = hasPieceSupplier;
        preloadPaths();
        generateSendableChooser();
    }

    private void preloadPaths() {
        preloadCenterLinePaths();
    }

    private void generateSendableChooser() {
        autoChooser.setDefaultOption("No Auto", Commands.none());
        
    }

    /**
     * Preloads the center line paths into the path cache.
     * These paths include shooting and skipping paths for each note on the center line.
     */
    private void preloadCenterLinePaths() {
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

            PathPlannerPath shootPiece = pathCache.get(shootingPathName + (shootingIndex - 1));
            
            if (shootingIndex == noteCount * 2) {
                fullPath.addCommands(AutoBuilder.followPath(shootPiece));
                return fullPath;
            }

            PathPlannerPath skipShot = pathCache.get(skippingPathName + i);
            PathPlannerPath getNextPiece = pathCache.get(shootingPathName + (shootingIndex));
            //? patritional command or no? not used so shouldn't matter :))
            fullPath.addCommands(
                    Commands.either(
                            AutoBuilder.followPath(shootPiece).andThen(AutoBuilder.followPath(getNextPiece)),
                            AutoBuilder.followPath(skipShot),
                            hasPieceSupplier));
        }
        return fullPath;
    }
}
