package frc.robot.commands.managers;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.logging.NT;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Trapper;
import frc.robot.util.Constants.TrapConstants;
import frc.robot.util.custom.SpeedAngleTriplet;
import monologue.Annotations.Log;

public class PieceControl {

    private Intake intake;
    private Indexer indexer;

    private Elevator elevator;
    private Trapper trapper;

    private ShooterCmds shooterCmds;

    private boolean shooterMode = true;

    // Between amp and trap
    private boolean ampPlaceMode = true;

    // State representing if we are trying to unstuck the elevator
    @Log
    private boolean elevatorDislodging = false;

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

    // TODO: only run angle reset when we are not using prepareSWDCommand
    public Command shootWhenReady(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        return Commands.waitUntil(shooterCmds.shooterCalc.readyToShootSupplier())
                .andThen(noteToShoot(poseSupplier, speedSupplier));
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

    public Command intakeToTrap() {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from trapper into indexer and then into 
        // shooter before stopping trapper and indexer
        return Commands.sequence(
            intake.inCommand(),
            trapper.intake(),
            indexer.toShooterSlow(),
            Commands.waitUntil(intake::getPossession),
            NT.getWaitCommand("intakeToTrap1"), // 0.5
            Commands.either(
                noteToTrap().andThen(noteToIndexer()), 
                noteToTrap(), 
                this::getShooterMode)
        );
    }

    public Command noteToIndexer() {
        return Commands.sequence(
            trapper.intake(),
            indexer.toShooterSlow(),
            NT.getWaitCommand("noteToIndexer1"), // 0.6
            indexer.stopCommand(),
            indexer.toElevatorSlow(),
            NT.getWaitCommand("noteToIndexer2"), // 0.07
            stopIntakeAndIndexer()
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
            dropPieceCommand(),
            stopAllMotors()
        );
    }

    public Command stopEjecting() {
        return Commands.parallel(
            elevator.toBottomCommand(),
            stopAllMotors()
        );
    }

    public Command dropPieceCommand() {
        return Commands.sequence(
            elevator.toDropCommand(),
            trapper.outtake(),
            NT.getWaitCommand("dropPieceCommand1"), // 0.5
            elevator.toBottomCommand()
        );
    }

    public Command indexCommand() {
        return elevator.toIndexCommand()
                .alongWith(intake.stopCommand());
    }

    public Command intakeAuto() {
        return Commands.sequence(
                intake.inCommand(),
                trapper.intake(),
                indexer.stopCommand());
    }

    public Command noteToTarget(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        // Defer the command to not require anyhing until it is actually ran
        // This is becuase placeTrap requires pivot, but shootWhenReady does not
        // If they are both required, then we would cancel any command that requires pivot
        // such as prepareSWDCommand
        return Commands.defer(() -> Commands.either(
            shootWhenReady(poseSupplier, speedSupplier),
            placeWhenReady(),
            this::getShooterMode
        ), this.getShooterMode()
            ? shootWhenReady(poseSupplier, speedSupplier).getRequirements() 
            : placeWhenReady().getRequirements());
    }

    private Command setDislodging(boolean dislodging) {
        return Commands.runOnce(() -> {
            elevatorDislodging = dislodging;
        });
    }

    // Same as normally setting elevator position but adds unstuck logic
    public Command setElevatorPosition(DoubleSupplier position) {
        return Commands.sequence(
            elevator.setPositionCommand(position, true),
            // Run until we are no longer in our unstucking state only if elevator actually gets stuck
            getUnstuck(position.getAsDouble()).onlyIf(elevator::getStuck).repeatedly().until(() -> !elevatorDislodging)
        );
    }

    public Command elevatorToTop() {
        return setElevatorPosition(() -> TrapConstants.ELEVATOR_TOP_LIMIT);
    }

    // Goes to amp position if the driver has toggled amp placement mode on, to trap otherwise
    public Command elevatorToPlacement() {
        return 
            Commands.defer(
                () -> 
                    Commands.either(
                        setElevatorPosition(NT.getSupplier("ampPosition")), 
                        setElevatorPosition(() -> TrapConstants.TRAP_PLACE_POS), 
                        () -> ampPlaceMode)
                    .andThen(prepPiece()),
                Set.of(elevator, trapper)
            );
    }

    public Command prepPiece() {
        return trapper.outtake(TrapConstants.PREP_PIECE_SECONDS);
    }

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

    public Command placeWhenReady() {
        return 
            Commands.defer(
                () -> Commands.sequence(
                    Commands.waitUntil(
                        () -> elevator.atPosition(
                            ampPlaceMode 
                            ? NT.getSupplier("ampPosition").getAsDouble()
                            : TrapConstants.TRAP_PLACE_POS)),
                    trapper.outtake(TrapConstants.OUTTAKE_SECONDS)
                    // shooterCmds.setTripletCommand(SpeedAngleTriplet.of(0, 0, 0))
                    //     .onlyIf(() -> !ampPlaceMode)
                ),
                Set.of(trapper)
            );
    }

    public Command sourceShooterIntake() {
        return Commands.sequence(
            shooterCmds.setTripletCommand(new SpeedAngleTriplet(-300.0, -300.0, 45.0)),
            indexer.toElevator(),
            trapper.outtake(3.0),
            indexCommand()
        ); 
    }

    public Command intakeToTrapper() { 
        return intake.inCommand()
                .alongWith(indexer.toElevator());
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

    public boolean getAmpPlaceMode() {
        return ampPlaceMode;
    }

    public void setAmpPlaceMode(boolean ampPlaceMode) {
        this.ampPlaceMode = ampPlaceMode;
    }

    public Command toggleAmpPlaceModeCommand() {
        return Commands.runOnce(() -> setAmpPlaceMode(!ampPlaceMode));
    }

    public Command setShooterModeCommand(boolean shooterMode) {
        return Commands.either(
                Commands.runOnce(() -> setShooterMode(shooterMode))
                    .andThen(
                        Commands.either(
                            Commands.either(
                                noteToIndexer(),
                                noteToTrap(),
                                () -> shooterMode), 
                            Commands.none(), 
                            intake::getPossession
                        )
                    ),
                Commands.none(),
                () -> getShooterMode() != shooterMode);
    }
}
