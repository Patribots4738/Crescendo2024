package frc.robot.commands.managers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.RobotContainer;
import frc.robot.commands.logging.NT;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Ampper;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.custom.ActiveConditionalCommand;
import frc.robot.util.custom.SpeedAngleTriplet;
import monologue.Annotations.Log;
import monologue.Logged;

public class PieceControl implements Logged {

    private Intake intake;
    private Indexer indexer;

    private Elevator elevator;
    private Ampper ampper;

    private ShooterCmds shooterCmds;

    private ColorSensor colorSensor;

    @Log
    private boolean shooterMode = false;

    // State representing if we are trying to unstuck the elevator
    @Log
    private boolean elevatorDislodging = false;
    @Log
    private boolean currentlyMovingNote = false;

    @Log
    private boolean hasPiece = false;

    public PieceControl(
            Intake intake,
            Indexer indexer,
            Elevator elevator,
            Ampper ampper,
            ShooterCmds shooterCmds,
            ColorSensor colorSensor) {
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        this.ampper = ampper;
        this.shooterCmds = shooterCmds;
        this.colorSensor = colorSensor;
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
            ampper.setCoastMode()
        );
    }

     public Command brakeIntakeAndIndexer() {
        return Commands.sequence(
            intake.setBrakeMode(),
            indexer.setBrakeMode(),
            ampper.setBrakeMode()
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
        // rotation and speed before sending note from ampper into indexer and then into 
        // shooter before stopping ampper and indexer
        return Commands.sequence(
                intake.inCommand(),
                ampper.intake(),
                indexer.toShooter(),
                NT.getWaitCommand("noteToShoot1"), // 0.7
                shooterCmds.getNoteTrajectoryCommand(poseSupplier, speedSupplier),
                NT.getWaitCommand("noteToShoot2"), // 0.4
                stopIntakeAndIndexer());
    }

    public Command noteToShootUsingSensor(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from ampper into indexer and then into 
        // shooter before stopping ampper and indexer
        return Commands.sequence(
                intake.inCommand(),
                ampper.intake(),
                indexer.toShooter(),
                shooterCmds.getNoteTrajectoryCommand(poseSupplier, speedSupplier),
                Commands.waitUntil(() -> !colorSensor.hasNote()),
                stopIntakeAndIndexer());
    }

    public Command intakeUntilNote() {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from ampper into indexer and then into 
        // shooter before stopping ampper and indexer
        return Commands.sequence(
            intake.inCommand(),
            ampper.intake(),
            indexer.toShooterSlow(),
            Commands.waitUntil(colorSensor::hasNote),
            stopIntakeAndIndexer()
        );
    }
    
    public Command intakeNote(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        return Commands.either(
            noteToShoot(poseSupplier, speedSupplier)
                .andThen(
                    Commands.defer(
                        () -> intakeNote(poseSupplier, speedSupplier), 
                        intakeUntilNote().getRequirements()
                    )
                ),
            intakeUntilNote(), 
            colorSensor::hasNote);
    }

    public Command noteToIndexer() {
        return Commands.sequence(
            intake.inCommand(),
            ampper.intake(),
            indexer.toShooterSlow(),
            Commands.waitUntil(colorSensor::hasNote),
            stopIntakeAndIndexer()
        );
    }

    public Command noteToTrap() {
        return Commands.sequence(
            ampper.outtake(),
            indexer.toElevator(),
            Commands.waitUntil(() -> !colorSensor.hasNote()),
            stopIntakeAndIndexer(),
            ampper.outtakeSlow(),
            NT.getWaitCommand("noteToTrap2"), // 0.5
            stopIntakeAndIndexer()
        );
    }

    public Command ejectNote() {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from ampper into indexer and then into 
        // shooter before stopping ampper and indexer
        return Commands.sequence(
            intake.outCommand(),
            indexer.toElevator(),
            ampper.outtake(),
            setHasPiece(false)
        );
    }

    public Command stopEjecting() {
        return Commands.parallel(
            elevator.toBottomCommand(),
            stopAllMotors()
        );
    }

    public Command panicEjectNote() {
        return ampper.intake().alongWith(indexer.toElevator()).alongWith(intake.outCommand());
    }

    public Command stopPanicEject() {
        return ampper.stopCommand().alongWith(indexer.stopCommand()).alongWith(intake.outCommand());
    }

    public Command intakeAuto() {
        return Commands.sequence(
                intake.inCommand(),
                ampper.intake(),
                indexer.toShooter());
    }

    public Command noteToTarget(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        // Defer the command to not require anyhing until it is actually ran
        // This is becuase placeTrap requires pivot, but shootWhenReady does not
        // If they are both required, then we would cancel any command that requires pivot
        // such as prepareSWDCommand
        return 
            new SelectiveConditionalCommand(
                Commands.race(shootWhenReady(poseSupplier, speedSupplier).andThen(setHasPiece(false)),Commands.waitUntil(() -> elevator.getDesiredPosition() > 0)),
                Commands.race(prepPiece().andThen(placeWhenReady()).andThen(setHasPiece(false)),Commands.waitUntil(() -> elevator.getDesiredPosition() <= 0)),
                () -> elevator.getDesiredPosition() <= 0
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private Command setDislodging(boolean dislodging) {
        return Commands.runOnce(() -> {
            elevatorDislodging = dislodging;
        });
    }

    // Same as normally setting elevator position but adds unstuck logic
    public Command setElevatorPosition(DoubleSupplier position) {
        return Commands.sequence(
            elevator.setPositionCommand(position, false)
            // Run until we are no longer in our unstucking state only if elevator actually gets stuck
            // getUnstuck(position.getAsDouble()).onlyIf(elevator::getStuck).repeatedly().until(() -> !elevatorDislodging)
        );
    }

    public Command elevatorToTop() {
        return setElevatorPosition(() -> ElevatorConstants.ELEVATOR_TOP_LIMIT);
    }
    public Command elevatorToBottom() {
        return setElevatorPosition(() -> ElevatorConstants.ELEVATOR_BOTTOM_LIMIT)
            .andThen(setPlaceWhenReadyCommand(false));
    }

    public Command prepPiece() {
        return Commands.sequence(
            ampper.intakeSlow(),
            NT.getWaitCommand("prepPiece"),
            ampper.stopCommand()
        );
    }

    // TODO: possible repurpose for getting unstuck from any position where we have unforeseen problems
    public Command getUnstuck(double desiredPose) {
        return 
            Commands.sequence(
                // Toggle this state to currently unstucking if we haven't already
                setDislodging(true),
                elevator.setPositionCommand(ElevatorConstants.UNSTUCK_POS),
                ampper.outtakeSlow(ElevatorConstants.UNSTUCK_OUTTAKE_TIME_SECONDS),
                elevator.setPositionCommand(desiredPose, true),
                // Toggle unstucking state to off if the elevator isn't actually stuck anymore
                setDislodging(false).onlyIf(() -> !elevator.getStuck())
            );
    }

    public Command elevatorToPlacement(boolean povLeftPosition) {
        return 
            Commands.sequence(
                stopIntakeAndIndexer(),
                Commands.either(
                    setElevatorPosition(() -> ElevatorConstants.NOTE_FIX_POS), 
                    setElevatorPosition(() -> ElevatorConstants.TRAP_PLACE_POS), 
                    () -> povLeftPosition),
                placeWhenReady().onlyIf(this::shouldPlaceWhenReady));
    }

    public Command placeWhenReady() {
        return
            new SelectiveConditionalCommand(
                Commands.sequence(
                    prepPiece(),
                    ampper.outtake(0.8),
                    elevatorToBottom()
                ),
                setPlaceWhenReadyCommand(true),
                elevator::atDesiredPosition);
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
            shooterCmds.setTripletCommand(new SpeedAngleTriplet(-300.0, -300.0, 60.0)),
            indexer.toElevator(),
            ampper.outtake()
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
                .alongWith(ampper.stopCommand());
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
  
    public Command setCurrentlyMovingNote(boolean currentlyMovingNote) {
        return Commands.runOnce(() -> this.currentlyMovingNote = currentlyMovingNote);
    }

    @Log
    private boolean desiredSide = false;

    public Command moveNote(boolean desiredSide) {
        return Commands.sequence(
                Commands.runOnce(() -> this.desiredSide = desiredSide),
                setCurrentlyMovingNote(true),
                Commands.either(
                    noteToIndexer(),
                    noteToTrap(),
                    () -> desiredSide))
                .finallyDo(() -> {
                    currentlyMovingNote = false;
                    if (desiredSide != shooterMode) {
                        moveNote(shooterMode).schedule();
                    }
                });
    }

    // Think of this parameter as the desired state of shooterMode.
    // We don't want to set the state until we are done with the command that moves the note
    // since we arent ready to do another command that moves the note until we are done with the first one
    public Command setShooterModeCommand(boolean newShooterMode) {
        return Commands.runOnce(() -> setShooterMode(newShooterMode))
                .andThen(
                    new SelectiveConditionalCommand(
                        // Fork off from the current sequence
                        // So that this button can continue to be spammed
                        // without waking up the baby
                        new ScheduleCommand(moveNote(newShooterMode)),
                        Commands.none(),
                        () -> !currentlyMovingNote)
                );
    }

    // Within a range of the [red circle](https://www.desmos.com/calculator/cu3ocssv5d)
    public Command getAutomaticShooterSpeeds(Supplier<Pose2d> robotPose) {
        return new ActiveConditionalCommand(
            Commands.runOnce(
                () -> shooterCmds.setSpeeds(ShooterConstants.DEFAULT_RPM), 
                shooterCmds.getShooter()
            ),
            shooterCmds.stopShooter(),
            () -> colorSensor.hasNote()
                && RobotContainer.distanceToSpeakerMeters < FieldConstants.AUTOMATIC_SHOOTER_DISTANCE_RADIUS)
            .onlyIf(() -> Robot.gameMode != GameMode.TEST);
    }
}
