package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.constants.Constants.AutoConstants;
import frc.robot.util.constants.Constants.FieldConstants;
import frc.robot.util.mod.PatriSendableChooser;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Robot.GameMode;
import monologue.Logged;
import monologue.Annotations.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

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
    private PatriSendableChooser<Command> autoChooser = new PatriSendableChooser<>();

    public static final ArrayList<Pose2d> AUTO_STARTING_POSITIONS = new ArrayList<Pose2d>();

    public static final HashMap<String, List<PathPlannerPath>> AUTO_PATHS = new HashMap<String, List<PathPlannerPath>>();
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
         * AutoBuilder::buildAutoChooser will load all autos in the deploy directory. Since the deploy
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
            // Load the auto and add it to the auto chooser
            Command auto = AutoBuilder.buildAuto(autoName);
            autoChooser.addOption(autoName, auto);
            // Load the auto and add it to the list of starting positions
            // for LPI
            Pose2d startingPosition = PathPlannerAuto.getStaringPoseFromAutoFile(autoName);
            PathPlannerStorage.AUTO_STARTING_POSITIONS.add(startingPosition);
            // Load the auto and add it to the list of paths 
            // for trajectory visualization
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            PathPlannerStorage.AUTO_PATHS.put(autoName, paths);
        }
        
        bindListener(getUpdatePathViewerCommand());
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }

    public String getSelectedAutoName() {
        return autoChooser.getSelectedName();
    }

    public void bindListener(Consumer<Command> consumer) {
        autoChooser.onChange(consumer);
    }

    public Command updatePathViewerCommand() {
        return Commands.either(
            Commands.runOnce(() -> {
                RobotContainer.field2d.getObject("path").setPoses(new ArrayList<>());
            }),
            Commands.runOnce(() -> {
                RobotContainer.field2d.getObject("path")
                    .setPoses(getAutoPoses(getSelectedAutoName()));
            }),
            () -> Robot.gameMode == GameMode.TELEOP
        ).ignoringDisable(true);
    }

    private Consumer<Command> getUpdatePathViewerCommand() {
        return (command) -> {
            updatePathViewerCommand().schedule();
        };
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
            
            if (i == FieldConstants.CENTER_NOTE_COUNT && goingDown || i == 1 && !goingDown || i == endingNote) {
                commandGroup.addCommands(
                    Commands.defer(() ->  AutoBuilder.followPath(shootNote), commandGroup.getRequirements())
                );
                break;
            }

            PathPlannerPath getNoteAfterShot = PathPlannerPath.fromPathFile(shootingLocation + " C" + (i + increment));
            PathPlannerPath skipNote = PathPlannerPath.fromPathFile("C" + i + " C" + (i + increment));
            
            Command shootAndMoveToNextNote = AutoBuilder.followPath(shootNote).andThen(AutoBuilder.followPath(getNoteAfterShot));
            // TODO: This one could be a pathfinder path that enables the moment we don't see a piece or simialar
            Command skipNoteCommand = AutoBuilder.followPath(skipNote);

            commandGroup.addCommands(
                Commands.defer(() -> 
                    Commands.either(
                        shootAndMoveToNextNote,
                        skipNoteCommand,
                        hasPieceSupplier), 
                        commandGroup.getRequirements())
            );
        }

        return commandGroup;
    }

    public Pose2d getPathEndPose(PathPlannerPath path) {
        List<Pose2d> poses = path.getPathPoses();
        Pose2d lastPose = poses.get(poses.size()-1);
        return lastPose;
    }

    public List<Pose2d> getAutoPoses(String name) {
        List<PathPlannerPath> paths = PathPlannerStorage.AUTO_PATHS.get(name);
        List<Pose2d> autoPoses = new ArrayList<>();
        if (paths == null) return autoPoses;
        // Flip for red alliance
        // and add all the poses to the list
        for (PathPlannerPath path : paths) {
            List<Pose2d> pathPoses = 
                Robot.isRedAlliance() 
                    ? path.flipPath().getPathPoses() 
                    : path.getPathPoses();
            autoPoses.addAll(pathPoses);
        }

        // We can only have up to 85 poses in the list, 
        // so we need to compress it if it's too large
        compressList(autoPoses, 84);

        return autoPoses;
    }

    private void compressList(List<Pose2d> poses, int maxSize) {
        int index = 0;
        while (poses.size() > maxSize && index < poses.size()) {
            if (index % 2 != 0) {
                poses.remove(index);
            } else {
                index++;
            }
        }
    }

    @Log
    Translation2d activePathEndPose = new Translation2d();
    public Translation2d getNextShotTranslation() {
        double[] activeTraj = NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("activePath").getDoubleArray(new double[0]);
        
        if (activeTraj.length == 0) return new Translation2d(0, 0);

        int activeTrajLength = activeTraj.length;
        double endX = activeTraj[activeTrajLength - 3];
        double endY = activeTraj[activeTrajLength - 2];
        Translation2d endPose = new Translation2d(endX, endY);

        this.activePathEndPose = endPose;
        return endPose;
    }
}
