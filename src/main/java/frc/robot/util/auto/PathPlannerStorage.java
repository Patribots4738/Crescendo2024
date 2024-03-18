package frc.robot.util.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.CameraConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.custom.PatriSendableChooser;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Robot.GameMode;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import monologue.Logged;
import monologue.Annotations.IgnoreLogged;
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
    private final BooleanSupplier colorSensorSupplier;
    
    @Log.NT
    private PatriSendableChooser<Command> autoChooser = new PatriSendableChooser<>();

    public static final ArrayList<Pose2d> AUTO_STARTING_POSITIONS = new ArrayList<Pose2d>();

    public static List<Pose2d> NOTE_POSES = FieldConstants.GET_CENTERLINE_NOTES();

    @IgnoreLogged
    private Swerve swerve;
    @IgnoreLogged
    private Limelight limelight;

    public static final PathConstraints PATH_CONSTRAINTS = 
        new PathConstraints(
            AutoConstants.MAX_SPEED_METERS_PER_SECOND, 
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, 
            AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
            AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final HashMap<String, List<PathPlannerPath>> AUTO_PATHS = new HashMap<String, List<PathPlannerPath>>();
    /**
     * Creates a new AutoPathStorage object.
     * @param hasPieceSupplier A supplier that returns whether or not the robot has a piece.
     *                         This could be a sensor, motor current, or other system.
     */
    public PathPlannerStorage(BooleanSupplier hasPieceSupplier, BooleanSupplier colorSensorSupplier, Swerve swerve, Limelight limelight) {
        this.hasPieceSupplier = hasPieceSupplier;
        this.colorSensorSupplier = colorSensorSupplier;
        this.swerve = swerve;
        this.limelight = limelight;
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
        System.out.println("Configured auto chooser");
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

    public PatriSendableChooser<Command> getAutoChooser() {
        return this.autoChooser;
    }

    public Command updatePathViewerCommand() {
        return Commands.either(
            Commands.runOnce(() -> {
                RobotContainer.field2d.getObject("path")
                    .setPoses(getAutoPoses(getSelectedAutoName()));
            }),
            Commands.runOnce(() -> {
                RobotContainer.field2d.getObject("path").setPoses(new ArrayList<>());
            }),
            () -> Robot.gameMode == GameMode.DISABLED
        ).ignoringDisable(true);
    }

    private Consumer<Command> getUpdatePathViewerCommand() {
        return (command) -> {
            updatePathViewerCommand().schedule();
        };
    }

    /**
     * IF USING OBJ DETECTION:
     * Given a starting note and an ending note, this method will generate a command
     * that uses the object detection on our limelight to 
     * If we see a piece, grab it and go to the nearest shooting position.
     * If we don't see a piece we try to look for the next one.
     * 
     * ELSE:
     * Given a starting note and an ending note, this method will generate a command
     * that uses the booleanSupplier hasPieceSupplier to determine whether or not the
     * robot has a piece. 
     * If we have a piece: come to wing, shoot it, and go to the next note.
     * If we don't have a piece: skip the shooting and go to the next note.
     * 
     * @param startingNote The note to start at (1-5)
     * @param endingNote   The note to end at (1-5)
     * @param swerve  The swerve subsystem to use.
     * @param limelight The limelight subsystem to use
     * @return  The command that will execute the logic for the given notes
     */
    public Command generateCenterLogic(int startingNote, int endingNote, Swerve swerve, Limelight limelight) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        boolean goingDown = startingNote < endingNote;
        int increment = goingDown ? 1 : -1;

        for (int i = startingNote; (goingDown && i <= endingNote) || (!goingDown && i >= endingNote); i += increment) {
            if (AutoConstants.USE_OBJECT_DETECTION) {
                commandGroup.addCommands(generateObjectDetectionCommand(i, endingNote, goingDown, commandGroup));
            } else {
                commandGroup.addCommands(generateNonObjectDetectionCommand(i, endingNote, goingDown, increment, commandGroup));
            }
        }

        return commandGroup;
    }

    /**
     * Generates a command for the object detection scenario.
     */
    private Command generateObjectDetectionCommand(int i, int endingNote, boolean goingDown, SequentialCommandGroup commandGroup) {
        int currentIndex = i - 1;
        int nextIndex = currentIndex + (goingDown ? 1 : -1);
        if ((goingDown && i < endingNote) || (!goingDown && i > endingNote)) {
            return Commands.defer(
                () -> Commands.either(
                    goToNote()
                        // This .onlyIf accounts for us trying to get a note but failing 
                        // and needing to go to the next one
                        .andThen(pathfindToShoot().onlyIf(colorSensorSupplier)
                            .andThen(pathfindToNextNote(nextIndex, goingDown))),
                    pathfindToNextNote(nextIndex, goingDown), 
                    () -> limelight.noteInVision(limelight.getResults())),
                commandGroup.getRequirements());
        } else {
            return Commands.defer(
                () -> Commands.either(
                    Commands.sequence(
                        goToNote(),
                        pathfindToShoot()
                    ), 
                    Commands.runOnce(swerve::stopDriving, swerve)
                        .andThen(
                            swerve.getScanCommand()
                            .until(() -> limelight.noteInVision(limelight.getResults()))
                            .andThen(
                                Commands.sequence(
                                    goToNote(),
                                    pathfindToShoot()
                                )
                            )
                        ),
                    () -> limelight.noteInVision(limelight.getResults())),
                commandGroup.getRequirements());
        }
    }

    /**
     * Generates a command for the non-object detection scenario.
     */
    private Command generateNonObjectDetectionCommand(int i, int endingNote, boolean goingDown, int increment, SequentialCommandGroup commandGroup) {
        String shootingLocation = (i < 3) ? "L" : (i == 3) ? "M" : "R";
        PathPlannerPath shootNote = PathPlannerPath.fromPathFile("C" + i + " " + shootingLocation);

        if (i == FieldConstants.CENTER_NOTE_COUNT && goingDown || i == 1 && !goingDown || i == endingNote) {
            return Commands.defer(() ->  AutoBuilder.followPath(shootNote), commandGroup.getRequirements());
        }

        PathPlannerPath getNoteAfterShot = PathPlannerPath.fromPathFile(shootingLocation + " C" + (i + increment));
        PathPlannerPath skipNote = PathPlannerPath.fromPathFile("C" + i + " C" + (i + increment));

        Command shootAndMoveToNextNote = AutoBuilder.followPath(shootNote)
            .andThen(AutoBuilder.followPath(getNoteAfterShot));
        // TODO: This one could be a pathfinder path that enables the moment we don't see a piece or simialar
        Command skipNoteCommand = AutoBuilder.followPath(skipNote);

        return Commands.defer(() -> 
            Commands.either(
                shootAndMoveToNextNote,
                skipNoteCommand,
                hasPieceSupplier), 
            commandGroup.getRequirements());
    }

    /**
     * Uses Pathplanner's pathfinding algorithm to go to the closest shooting position 
     * from the swerve subsystem's curent position
     * 
     * @param swerve  The swerve subsystem to use.
     * @return  The command that will pathfind to the shooting pose
     */
    public Command pathfindToShoot() {
        return 
            AutoBuilder.pathfindToPose(
                PoseCalculations.getClosestShootingPose(swerve.getPose()), 
                PATH_CONSTRAINTS,
                0)
            .andThen(NamedCommands.getCommand("ShootInstantly"))
            .raceWith(Commands.waitSeconds(1.5).andThen(
                    Commands.waitUntil(() -> !colorSensorSupplier.getAsBoolean())
                )
            );
    }

    /**
     * Uses Pathplanner's pathfinding algorithm to go to the next note in our centerline logic 
     * so that we can see if it is there
     * 
     * @param index The supplier for the next note index so that we know which
     *              note to pathfind to next
     * @return  The command that will pathfind towards the next note
     */
    public Command pathfindToNextNote(int index, boolean goingDown) {
        Translation2d searchSpot = new Translation2d(
            NOTE_POSES.get(index).getX() + 
                (Robot.isRedAlliance() 
                ? AutoConstants.PIECE_SEARCH_OFFSET_METERS
                : -AutoConstants.PIECE_SEARCH_OFFSET_METERS), 
            NOTE_POSES.get(index).getY());

        return 
            // Race the path and vision
            // once we catch sight of a note, we can go straight to it.
            Commands.race(
                AutoBuilder.pathfindToPose(
                    new Pose2d(
                        searchSpot,
                        // Angle the robot slightly towards the note
                        Rotation2d.fromDegrees(
                            Robot.isRedAlliance() ^ goingDown 
                                ? -CameraConstants.LL2_HORIZONTAL_FOV/4.0 
                                :  CameraConstants.LL2_HORIZONTAL_FOV/4.0
                        ).plus(Rotation2d.fromRadians(Robot.isRedAlliance() ? 0 : Math.PI))),
                    PATH_CONSTRAINTS,
                    0
                ),
                Commands.waitUntil(() -> limelight.noteInVision(limelight.getResults()))
            );
    }

    /**
     * Uses a custom chase command to drive towards a predetermined note position
     * Primarily for testing use without real notes
     * 
     * @param swerve  The swerve subsystem to use.
     * @param currentIndex The current index of the note so that we can get a predetermined note
     *                      pose without our vision
     * @return  The command that drives to a preset note position
     */
    public Command goToNote() {
        return swerve.getChaseCommand( 
            () ->
                new Pose2d(
                    limelight.getNotePose2d().getTranslation(), 
                    Rotation2d.fromRadians(Robot.isRedAlliance() ? 0 : Math.PI)),
            () -> 
                colorSensorSupplier.getAsBoolean() 
                // Add 20 inches of cushion since we can't get penalized until we go 35 inches past the center line (bumpers fully over)
                // Keep in mind this is the note itself being 35 inches, the robot can only go 35/2 inches
                // since the pose is from the center but the note is from the edge (since the intake gets it)
                || (Robot.isBlueAlliance() && limelight.getNotePose2d().getTranslation().getX() > FieldConstants.CENTERLINE_X + Units.inchesToMeters(20))
                || (Robot.isRedAlliance() && limelight.getNotePose2d().getTranslation().getX() < FieldConstants.CENTERLINE_X - Units.inchesToMeters(20)))
            .alongWith(NamedCommands.getCommand("ToIndexer"));
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
    
        return autoPoses;
    }

    public Translation2d getNextShotTranslation() {
        double[] activeTraj = NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("activePath").getDoubleArray(new double[0]);
        
        if (activeTraj.length == 0) return new Translation2d(0, 0);

        int activeTrajLength = activeTraj.length;
        double endX = activeTraj[activeTrajLength - 3];
        double endY = activeTraj[activeTrajLength - 2];
        Translation2d endTranslation = new Translation2d(endX, endY);

        return endTranslation;
    }
}
