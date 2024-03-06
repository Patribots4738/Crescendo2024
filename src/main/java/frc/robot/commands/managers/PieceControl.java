package frc.robot.commands.managers;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.RobotContainer;
import frc.robot.commands.logging.NT;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Trapper;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.Constants.TrapConstants;
import frc.robot.util.custom.ActiveConditionalCommand;
import frc.robot.util.custom.SpeedAngleTriplet;
import monologue.Logged;
import monologue.Annotations.Log;

public class PieceControl implements Logged {

    private Intake intake;
    private Indexer indexer;

    private Elevator elevator;
    private Trapper trapper;

    private ShooterCmds shooterCmds;

    @Log
    private boolean shooterMode = true;

    // State representing if we are trying to unstuck the elevator
    @Log
    private boolean elevatorDislodging = false;
    @Log
    private boolean readyToMoveNote = true;

    // State representing if we are ready to place with trapper
    @Log
    private boolean readyToPlace = false;

    @Log
    private boolean hasPiece = false;

    public PieceControl(
            Intake intake,
            Indexer indexer,
            Elevator elevator,
            Trapper trapper,
            ShooterCmds shooterCmds) {
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        this.trapper = trapper;
        this.shooterCmds = shooterCmds;
    }

    public Command stopAllMotors() {
        return Commands.parallel(
                stopIntakeAndIndexer(),
                shooterCmds.stopShooter()).ignoringDisable(true);
    }

    public Command coastIntakeAndIndexer() {
        return Commands.sequence(
            intake.setCoastMode(),
            indexer.setCoastMode(),
            trapper.setCoastMode()
        );
    }

     public Command brakeIntakeAndIndexer() {
        return Commands.sequence(
            intake.setBrakeMode(),
            indexer.setBrakeMode(),
            trapper.setBrakeMode()
        );
    }

    // TODO: only run angle reset when we are not using prepareSWDCommand
    public Command shootWhenReady(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        return Commands.waitUntil(shooterCmds.shooterCalc.readyToShootSupplier())
                .andThen(noteToShoot(poseSupplier, speedSupplier));
    }

    public Command shootPreload() {
        return Commands.waitUntil(shooterCmds.shooterCalc.readyToShootSupplier())
                .andThen(intakeAuto());
    }

    // TODO: Possibly split this into two commands where one sends to shooter
    // without waiting
    public Command noteToShoot(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from trapper into indexer and then into 
        // shooter before stopping trapper and indexer
        return Commands.sequence(
                intake.inCommand(),
                trapper.intake(),
                indexer.toShooter(),
                NT.getWaitCommand("noteToShoot1"), // 0.7
                shooterCmds.getNoteTrajectoryCommand(poseSupplier, speedSupplier),
                NT.getWaitCommand("noteToShoot2"), // 0.4
                stopIntakeAndIndexer());
    }

    public Command intakeNote() {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from trapper into indexer and then into 
        // shooter before stopping trapper and indexer
        return Commands.sequence(
            intake.inCommand(),
            trapper.intake(),
            indexer.toShooterSlow(),
            Commands.waitUntil(intake::getPossession),
            setHasPiece(true),
            NT.getWaitCommand("intakeToTrap1"), // 0.5
            // Use the trap to index it the first time around
            noteToTrap(),
            // Then send it to its desired location
            moveNote()
        );
    }  

    public Command noteToIndexer() {
        return Commands.sequence(
            intake.inCommand(),
            trapper.intake(),
            indexer.toShooterSlow(),
            NT.getWaitCommand("noteToIndexer1"), // 0.7
            trapper.stopCommand(),
            indexer.toElevatorSlow(),
            NT.getWaitCommand("noteToIndexer2"), // 0.07
            stopIntakeAndIndexer()
        );
    }

    public Command toIndexerAuto() {
        return Commands.sequence(
            trapper.intake(),
            intake.inCommand()
            
        );
    }

    

    public Command noteToTrap() {
        return Commands.sequence(
            trapper.outtake(),
            indexer.toElevator(),
            NT.getWaitCommand("noteToTrap1"), // 0.2
            stopIntakeAndIndexer(),
            trapper.outtakeSlow(),
            NT.getWaitCommand("noteToTrap2"), // 0.5
            stopIntakeAndIndexer()
        );
    }

    public Command ejectNote() {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from trapper into indexer and then into 
        // shooter before stopping trapper and indexer
        return Commands.sequence(
            intake.outCommand(),
            indexer.toElevator(),
            trapper.outtake()
        );
    }

    public Command stopEjecting() {
        return Commands.parallel(
            elevator.toBottomCommand(),
            stopAllMotors()
        );
    }

    public Command panicEjectNote() {
        return trapper.intake().andThen(indexer.toElevator());
    }

    public Command stopPanicEject() {
        return trapper.stopCommand().andThen(indexer.stopCommand());
    }

    public Command intakeAuto() {
        return Commands.sequence(
                intake.inCommand(),
                trapper.intake(),
                indexer.toShooter());
    }

    public Command noteToTarget(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        // Defer the command to not require anyhing until it is actually ran
        // This is becuase placeTrap requires pivot, but shootWhenReady does not
        // If they are both required, then we would cancel any command that requires pivot
        // such as prepareSWDCommand
        return 
            new SelectiveConditionalCommand(
                shootWhenReady(poseSupplier, speedSupplier),
                placeWhenReady(),
                () -> elevator.getDesiredPosition() == 0
            ).andThen(setHasPiece(false));
    }

    private Command setDislodging(boolean dislodging) {
        return Commands.runOnce(() -> {
            elevatorDislodging = dislodging;
        });
    }

    // Same as normally setting elevator position but adds unstuck logic
    public Command setElevatorPosition(DoubleSupplier position) {
        return Commands.sequence(
            setReadyToPlaceCommand(false),
            elevator.setPositionCommand(position, true)
            // Run until we are no longer in our unstucking state only if elevator actually gets stuck
            // getUnstuck(position.getAsDouble()).onlyIf(elevator::getStuck).repeatedly().until(() -> !elevatorDislodging)
        );
    }

    public Command elevatorToTop() {
        return setElevatorPosition(() -> TrapConstants.ELEVATOR_TOP_LIMIT);
    }

    private void setReadyToPlace(boolean readyToPlace) {
        this.readyToPlace = readyToPlace;
    }

    private Command setReadyToPlaceCommand(boolean readyToPlace) {
        return Commands.runOnce(() -> setReadyToPlace(readyToPlace));
    }

    public Command elevatorToBottom() {
        return setElevatorPosition(() -> TrapConstants.ELEVATOR_BOTTOM_LIMIT)
            .andThen(setPlaceWhenReadyCommand(false));
    }

    public Command prepPiece() {
        return Commands.sequence(
            trapper.intakeSlow(),
            NT.getWaitCommand("prepPiece"),
            trapper.stopCommand()
        );
    }

    // TODO: possible repurpose for getting unstuck from any position where we have unforeseen problems
    public Command getUnstuck(double desiredPose) {
        return 
            Commands.sequence(
                // Toggle this state to currently unstucking if we haven't already
                setDislodging(true),
                elevator.setPositionCommand(TrapConstants.UNSTUCK_POS),
                trapper.outtakeSlow(TrapConstants.UNSTUCK_OUTTAKE_TIME_SECONDS),
                elevator.setPositionCommand(desiredPose, true),
                // Toggle unstucking state to off if the elevator isn't actually stuck anymore
                setDislodging(false).onlyIf(() -> !elevator.getStuck())
            );
    }

    // TODO: remove defer when amp position is finalized
    public Command elevatorToPlacement(boolean amp) {
        return 
            Commands.defer(
                () -> 
                    Commands.sequence(
                        Commands.either(
                            setElevatorPosition(NT.getSupplier("ampPosition")), 
                            setElevatorPosition(() -> TrapConstants.TRAP_PLACE_POS), 
                            () -> amp),
                        prepPiece(),
                        setReadyToPlaceCommand(true),
                        placeWhenReady().onlyIf(this::shouldPlaceWhenReady)
                    ),
                Set.of(elevator, trapper)
            );
    }

    public Command placeWhenReady() {
        return 
            new SelectiveConditionalCommand(
                Commands.sequence(
                    Commands.sequence(
                        trapper.outtake(),
                        NT.getWaitCommand("placeOuttake"),
                        trapper.stopCommand(),
                        elevatorToBottom()
                    )),
                setPlaceWhenReadyCommand(true),
                () -> readyToPlace);
    }

    @Log
    private boolean placeWhenReady = false;

    public boolean shouldPlaceWhenReady() {
        return placeWhenReady;
    }

    public Command setPlaceWhenReadyCommand(boolean placeWhenReady) {
        return Commands.runOnce(() -> this.placeWhenReady = placeWhenReady);
    }

    public Command sourceShooterIntake(BooleanSupplier holdingButton) {
        return Commands.sequence(
            shooterCmds.setTripletCommand(new SpeedAngleTriplet(-300.0, -300.0, 45.0)),
            indexer.toElevator(),
            trapper.outtake()
        ).onlyWhile(holdingButton)
        .andThen(
            Commands.either(
                stopAllMotors(),
                noteToTrap(),
                this::getShooterMode));
    }

    public Command stopIntakeAndIndexer() {
        return intake.stopCommand()
                .alongWith(indexer.stopCommand())
                .alongWith(trapper.stopCommand());
    }

    public boolean getShooterMode() {
        return shooterMode;
    }

    public void setShooterMode(boolean shooterMode) {
        this.shooterMode = shooterMode;
    }

    public Command setHasPiece(boolean hasPiece) {
        return Commands.runOnce(() -> this.hasPiece = hasPiece);
    }
  
    public Command setReadyToMoveNote(boolean readyToMove) {
        return Commands.runOnce(() -> this.readyToMoveNote = readyToMove);
    }

    public boolean readyToMoveNote() {
        return readyToMoveNote;
    }

    public Command moveNote() {
        return Commands.sequence(
                setReadyToMoveNote(false),
                Commands.either(
                    noteToIndexer(),
                    noteToTrap(),
                    this::getShooterMode),
                setReadyToMoveNote(true));
    }

    public Command moveNoteIfReady() {
        return Commands.either(
                moveNote(),
                Commands.none(),
                // TODO: Make intake.getPossesion more of a (note in robot) bool
                () -> intake.getPossession() && readyToMoveNote);
    }

    // Think of this parameter as the desired state of shooterMode.
    // We don't want to set the state until we are done with the command that moves the note
    // since we arent ready to do another command that moves the note until we are done with the first one
    public Command setShooterModeCommand(boolean shooterMode) {
        return Commands.runOnce(() -> setShooterMode(shooterMode))
                .andThen(
                    Commands.either(
                        noteToIndexer(),
                        noteToTrap(),
                        () -> shooterMode)
                );
    }

    // Within a range of the [red circle](https://www.desmos.com/calculator/cu3ocssv5d)
    public Command getAutomaticShooterSpeeds(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speedSupplier) {
        return new ActiveConditionalCommand(
            Commands.runOnce(() -> shooterCmds.setSpeeds(ShooterConstants.DEFAULT_RPM), 
            shooterCmds.getShooter()),
            shooterCmds.stopShooter().onlyIf(() -> Robot.gameMode != GameMode.TEST),
            () -> shooterMode 
                && hasPiece 
                && RobotContainer.distanceToSpeakerMeters < FieldConstants.AUTOMATIC_SHOOTER_DISTANCE_RADIUS 
                && Robot.gameMode != GameMode.TEST);
    }
}
