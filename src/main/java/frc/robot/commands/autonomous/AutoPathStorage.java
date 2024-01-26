package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.Constants.FieldConstants;

import java.util.function.BooleanSupplier;

/**
 * This file represents all of the auto paths that we will have
 * They will be primarily comiled through
 * PathPlannerTrajectory.importChoreoPaths,
 * with each segment having its own method to make sure that the modularity
 * stays clean
 */
public class AutoPathStorage {

    private final BooleanSupplier intakeHasPieceSupplier;

    public AutoPathStorage(BooleanSupplier intakeHasPieceSupplier) {
        this.intakeHasPieceSupplier = intakeHasPieceSupplier;
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
    // TODO: Think about the logistics of having 
    // each and every PathPLannerPath.fromChoreoTrajectory be loaded in the constructor
    // and stored in a hashmap, array, or something else...
    public Command generateCenterLineComplete(int startingIndex, int noteCount, boolean goingUp) {
        SequentialCommandGroup fullPath = new SequentialCommandGroup();

        String pathName = "C1-5";    
        if (goingUp){
            startingIndex = Math.abs(startingIndex-6);
            pathName = "C5-1";
        }

        for (int i = startingIndex; i <= noteCount; i++) {

            int index = (i * 2);

            PathPlannerPath shootPiece = PathPlannerPath.fromChoreoTrajectory(pathName + "S." + (index - 1));

            if (index == noteCount * 2) {
                fullPath.addCommands(AutoBuilder.followPath(shootPiece));
                return fullPath;
            }   

            PathPlannerPath getNextPiece = PathPlannerPath.fromChoreoTrajectory(pathName + "S." + (index));
            PathPlannerPath skipShot = PathPlannerPath.fromChoreoTrajectory(pathName + "." + i);

            fullPath.addCommands(
                    Commands.either(
                            AutoBuilder.followPath(shootPiece).andThen(AutoBuilder.followPath(getNextPiece)),
                            AutoBuilder.followPath(skipShot),
                            intakeHasPieceSupplier));
        }
        return fullPath;
    }
}
