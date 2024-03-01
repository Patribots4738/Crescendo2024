package frc.robot.commands.managers;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.logging.NT;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.elevator.Trapper;
import frc.robot.util.Constants.TrapConstants;
import frc.robot.util.custom.SpeedAngleTriplet;
import frc.robot.subsystems.elevator.Elevator;

public class PieceControl {

    private Intake intake;
    private Indexer indexer;

    private Elevator elevator;
    private Trapper trapper;

    private ShooterCmds shooterCmds;

    private boolean shooterMode = true;

    // State representing if we are trying to unstuck the elevator
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
                Commands.waitSeconds(NT.getSupplier("noteToShoot1").getAsDouble()), // 0.7
                shooterCmds.getNoteTrajectoryCommand(poseSupplier, speedSupplier),
                Commands.waitSeconds(NT.getSupplier("noteToShoot2").getAsDouble()), // 0.4
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
            Commands.waitSeconds(NT.getSupplier("intakeToTrap1").getAsDouble()), // 0.5
            noteToTrap(),
            noteToIndexer()
        );
    }

    public Command noteToIndexer() {
        return Commands.sequence(
            trapper.intake(),
            indexer.toShooterSlow(),
            Commands.waitSeconds(NT.getSupplier("noteToIndexer1").getAsDouble()), // 0.6
            indexer.stopCommand(),
            indexer.toElevatorSlow(),
            Commands.waitSeconds(NT.getSupplier("noteToIndexer2").getAsDouble()), // 0.07
            stopIntakeAndIndexer()
        );
    }

    public Command noteToTrap() {
        return Commands.sequence(
            trapper.outtake(),
            indexer.toElevator(),
            Commands.waitSeconds(NT.getSupplier("noteToTrap1").getAsDouble()), // 0.2
            stopIntakeAndIndexer(),
            trapper.outtakeSlow(),
            Commands.waitSeconds(NT.getSupplier("noteToTrap2").getAsDouble()), // 0.5
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
            Commands.waitSeconds(NT.getSupplier("dropPieceCommand1").getAsDouble()), // 0.5
            elevator.toBottomCommand()
        );
    }

    public Command indexCommand() {
        return elevator.toIndexCommand()
                .alongWith(intake.stopCommand(), trapper.stopCommand());
    }

    public Command intakeAuto() {
        return Commands.sequence(
                intake.inCommand(),
                trapper.intake(),
                indexer.stopCommand());
    }

    public Command noteToTarget(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        // maybe make setPosition a command ORR Make the Elevator Command
        return Commands.either(
            shootWhenReady(poseSupplier, speedSupplier),
            placeTrap(),
            this::getShooterMode
        );
    }

    private Command toggleStuck() {
        return Commands.runOnce(() -> {
            elevatorDislodging = !elevatorDislodging;
        });
    }

    // Same as normally setting elevator position but adds unstuck logic
    public Command setElevatorPosition(double position) {
        return Commands.sequence(
            elevator.setPositionCommand(position, true),
            // Run until we are no longer in our unstucking state only if elevator actually gets stuck
            getUnstuck(position).onlyIf(elevator::getStuck).repeatedly().until(() -> !elevatorDislodging)
        );
    }

    // Same as above
    public Command elevatorToTop() {
        return setElevatorPosition(TrapConstants.TRAP_PLACE_POS);
    }

    public Command elevatorToAmp() {
        return setElevatorPosition(TrapConstants.AMP_PLACE_POS);
    }

    public Command getUnstuck(double desiredPose) {
        return 
            Commands.sequence(
                // Toggle this state to currently unstucking if we haven't already
                toggleStuck().onlyIf(() -> !elevatorDislodging),
                elevator.setPositionCommand(TrapConstants.UNSTUCK_POS),
                trapper.outtakeSlow(),
                Commands.waitSeconds(TrapConstants.UNSTUCK_OUTTAKE_TIME_SECONDS),
                trapper.stopCommand(),
                elevator.setPositionCommand(desiredPose, true),
                // Toggle unstucking state to off if the elevator isn't actually stuck anymore
                toggleStuck().onlyIf(() -> !elevator.getStuck())
            );
    }

    public Command placeTrap() {
        return shooterCmds.setTripletCommand(SpeedAngleTriplet.of(0.0,0.0, 60.0)).alongWith(
            Commands.sequence(
                elevatorToTop(),
                trapper.intake(),
                Commands.waitSeconds(0.3),
                trapper.stopCommand(),
                Commands.waitSeconds(1),
                trapper.outtake(),
                Commands.waitSeconds(TrapConstants.OUTTAKE_SECONDS),
                trapper.stopCommand()
            )
        ).andThen(
            shooterCmds.setTripletCommand(SpeedAngleTriplet.of(0.0,0.0, 0.0))
        );
    }

    public Command sourceShooterIntake() {
        return Commands.sequence(
            shooterCmds.setTripletCommand(new SpeedAngleTriplet(-300.0, -300.0, 45.0)),
            indexer.toElevator(),
            trapper.outtake(),
            Commands.waitSeconds(3),
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

    public Command setShooterModeCommand(boolean shooterMode) {
        return Commands.runOnce(() -> setShooterMode(shooterMode));
    }
}
