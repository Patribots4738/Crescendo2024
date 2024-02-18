package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.FieldConstants;
import monologue.Logged;
import monologue.Annotations.Log;

import java.util.function.BooleanSupplier;

/**
 * This file represents all of the auto paths that we will have
 * They will be primarily comiled through
 * PathPlannerTrajectory.importChoreoPaths,
 * with each segment having its own method 
 * to make sure that the modularity stays clean
 */
public class PathPlannerStorage implements Logged {

    private final BooleanSupplier hasPieceSupplier;
    @Log.NT
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * Creates a new AutoPathStorage object.
     * @param hasPieceSupplier A supplier that returns whether or not the robot has a piece.
     *                         This could be a sensor, motor current, or other system.
     */
    public PathPlannerStorage(BooleanSupplier hasPieceSupplier) {
        this.hasPieceSupplier = hasPieceSupplier;
    }

    public void configureAutoChooser() {
        /**
         * Warning
         * 
         * This method will load all autos in the deploy directory. Since the deploy
         * process does not automatically clear the deploy directory, old auto files
         * that have since been deleted from the project could remain on the RIO,
         * therefore being added to the auto chooser.
         * 
         * To remove old options, the deploy directory will need to be cleared manually
         * via SSH, WinSCP, reimaging the RIO, etc.
         */

        // Good news! 
        // This auto caches our paths so we don't need to manually load them
        for (String autoName : AutoConstants.AUTO_NAMES) {
            autoChooser.addOption(autoName, AutoBuilder.buildAuto(autoName));
        }
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }

    /**
     * Given a starting note and an ending note, this method will generate a command
     * that uses the booleanSupplier hasPieceSupplier to determine whether or not the
     * robot has a piece. 
     * If we have a piece: come to wing, shoot it, and go to the next note.
     * If we don't have a piece: skip the shooting and go to the next note.
     * 
     * @param startingNote The note to start at (1-5)
     * @param endingNote   The note to end at (1-5)
     * @param goingDown  If true, increment from startingNote to endingNote. If false, decrement from startingNote to endingNote.
     * @return             The command that will execute the logic for the given notes
     */
    public Command generateCenterLogic(int startingNote, int endingNote) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        boolean goingDown = startingNote < endingNote;

        int increment = goingDown ? 1 : -1;

        for (int i = startingNote; (goingDown && i <= endingNote) || (!goingDown && i >= endingNote); i += increment) {
            String shootingLocation = 
                (i < 3) 
                    ? "L" 
                    : (i == 3) 
                        ? "M" 
                        : "R";

            PathPlannerPath shootNote = PathPlannerPath.fromPathFile("C" + i + " " + shootingLocation);

            if (i == FieldConstants.CENTER_NOTE_COUNT && goingDown || i == 1 && !goingDown) {
                commandGroup.addCommands(
                    AutoBuilder.followPath(shootNote)
                );
                break;
            }

            PathPlannerPath getNoteAfterShot = PathPlannerPath.fromPathFile(shootingLocation + " C" + (i + increment));
            PathPlannerPath skipNote = PathPlannerPath.fromPathFile("C" + i + " C" + (i + increment));

            Command shootAndMoveToNextNote = AutoBuilder.followPath(shootNote).andThen(AutoBuilder.followPath(getNoteAfterShot));
            // TODO: This one could be a pathfinder path that enables the moment we don't see a piece or simialar
            Command skipNoteCommand = AutoBuilder.followPath(skipNote);

            commandGroup.addCommands(
                Commands.either(
                    shootAndMoveToNextNote,
                    skipNoteCommand,
                    hasPieceSupplier
                )
            );
        }

        return commandGroup;
    }
}
