package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.BooleanSupplier;

/**
 * This file represents all of the auto paths that we will have
 * They will be primarily comiled through PathPlannerTrajectory.importChoreoPaths, 
 * with each segment having its own method to make sure that the modularity stays clean
 */
public class AutoPathStorage {

    private final BooleanSupplier intakeHasPieceSupplier;

    public AutoPathStorage(BooleanSupplier intakeHasPiece) {
        this.intakeHasPieceSupplier = intakeHasPiece;
    }

    public Command generateFullPath(boolean goingDown, int startingIndex) {
        SequentialCommandGroup fullPath = new SequentialCommandGroup();

        final int NOTE_COUNT = 5;
        
        if (goingDown) {
            for (int i = startingIndex; i <= NOTE_COUNT; i++) {

                int index = i * 2;

                PathPlannerPath shootPiece = PathPlannerPath.fromChoreoTrajectory("C1-5S."+(index-1));

                if (index == NOTE_COUNT*2) {
                    fullPath.addCommands(AutoBuilder.followPath(shootPiece));
                    return fullPath;
                }

                PathPlannerPath getNextPiece = PathPlannerPath.fromChoreoTrajectory("C1-5S."+(index));
                PathPlannerPath skipShot = PathPlannerPath.fromChoreoTrajectory("C1-5."+i);
                
                fullPath.addCommands(
                    Commands.either(
                        AutoBuilder.followPath(shootPiece).andThen(AutoBuilder.followPath(getNextPiece)), 
                        AutoBuilder.followPath(skipShot), 
                        intakeHasPieceSupplier)
                    );
            }
        } else {
            for (int i = startingIndex; i > 0; i--) {
                PathPlannerPath shootPiece = PathPlannerPath.fromChoreoTrajectory("C5-1S."+(i-1)*2);
                PathPlannerPath getNextPiece = PathPlannerPath.fromChoreoTrajectory("C5-1S."+(i*2));
                PathPlannerPath skipShot = PathPlannerPath.fromChoreoTrajectory("C5-1."+i);
                
                fullPath.addCommands(
                    Commands.either(
                        AutoBuilder.followPath(shootPiece).andThen(AutoBuilder.followPath(getNextPiece)), 
                        AutoBuilder.followPath(skipShot), 
                        intakeHasPieceSupplier)
                    );
            }
        }

        return fullPath;
    }
}
