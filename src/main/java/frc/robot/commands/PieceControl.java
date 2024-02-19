package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.elevator.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.SpeedAngleTriplet;
import frc.robot.util.Constants.TrapConstants;

public class PieceControl {

    private Intake intake;
    private Indexer indexer;

    private Elevator elevator;
    private Claw claw;

    private ShooterCalc shooterCalc;

    private boolean shooterMode;

    public PieceControl(
            Intake intake,
            Indexer indexer,
            Elevator elevator,
            Claw claw,
            ShooterCalc shooterCalc) {
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        this.claw = claw;
        this.shooterCalc = shooterCalc;
    }

    public Command stopAllMotors() {
        return Commands.parallel(
                intake.stopCommand(),
                indexer.stopCommand(),
                elevator.stopCommand(),
                claw.stopCommand());
    }

    public Command shootWhenReady(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        return Commands.waitUntil(shooterCalc.readyToShootSupplier())
                .andThen(noteToShoot())
                    .alongWith(shooterCalc.getNoteTrajectoryCommand(poseSupplier, speedSupplier));
    }

    // TODO: Possibly split this into two commands where one sends to shooter
    // without waiting
    public Command noteToShoot() {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from claw into indexer and then into 
        // shooter before stopping claw and indexer
        return Commands.sequence(
                intake.inCommand(),
                claw.intake(),
                indexer.toShooter(),
                Commands.waitSeconds(1.5),
                stopAllMotors());

    }

    public Command noteToTrap() {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from claw into indexer and then into 
        // shooter before stopping claw and indexer
        return Commands.sequence(
                intake.inCommand(),
                claw.intake(),
                indexer.stopCommand(),
                Commands.waitSeconds(.75),
                stopAllMotors());

    }

    public Command toggleIn() {
        return Commands.either(
            // onTrue,
            noteToShoot(),
            // onFalse,
            stopIntakeAndIndexer(),
            // condition
            intake::isStopped
        );
    }

    public Command toggleOut() {
        return Commands.either(
            // onTrue,
            ejectNote(),
            // onFalse,
            stopIntakeAndIndexer(),
            // condition
            intake::isStopped
        );
    }

    public Command ejectNote() {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from claw into indexer and then into 
        // shooter before stopping claw and indexer
        return Commands.sequence(
                intake.outCommand(),
                claw.outtake(),
                indexer.toElevator(),
                Commands.waitSeconds(.75),
                stopAllMotors());

    }

    public Command noteToTarget(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        // maybe make setPosition a command ORR Make the Elevator Command
        return Commands.either(
            shootWhenReady(poseSupplier, speedSupplier),
            elevatorPlacementCommand(),
            this::getShooterMode
        );
    }

    public Command elevatorPlacementCommand() {
        return Commands.sequence(
            elevator.toTopCommand(),
            claw.placeCommand(),
            Commands.waitSeconds(TrapConstants.OUTTAKE_SECONDS),
            elevator.toBottomCommand()
        );
    }

    public Command sourceShooterIntake() {
        return Commands.sequence(
            Commands.runOnce(() -> shooterCalc.setTriplet(new SpeedAngleTriplet(-300.0, -300.0, 45.0))),
            indexer.toElevator(),
            claw.outtake(),
            Commands.waitSeconds(3),
            stopAllMotors()
        ); 
    }

    public Command intakeToClaw() { 
        return intake.inCommand()
                .alongWith(indexer.toElevator());
    }

    public Command stopIntakeAndIndexer() {
        return intake.stopCommand()
                .alongWith(indexer.stopCommand())
                .alongWith(claw.stopCommand());
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
