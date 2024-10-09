package frc.robot.commands.managers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.RobotContainer;
import frc.robot.commands.logging.NT;
import frc.robot.subsystems.colorsensor.PicoColorSensor;
import frc.robot.subsystems.ampper.Elevator;
import frc.robot.subsystems.intake.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ampper.Ampper;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.custom.ActiveConditionalCommand;
import frc.robot.util.custom.SelectiveConditionalCommand;
import frc.robot.util.custom.SpeedAngleTriplet;

public class PieceControl {
    
    private Intake intake;
    
    private Indexer indexer;

    private Elevator elevator;

    private Ampper ampper;

    private ShooterCmds shooterCmds;

    private PicoColorSensor piPico;

    @AutoLogOutput (key = "Managers/PieceControl/ShooterMode")
    private boolean shooterMode = false;

    // State representing if we are trying to unstuck the elevator
    @AutoLogOutput (key = "Managers/PieceControl/ElevatorDislodging")
    private boolean elevatorDislodging = false;

    @AutoLogOutput (key = "Managers/PieceControl/MovingNote")
    private boolean currentlyMovingNote = false;

    @AutoLogOutput (key = "Managers/PieceControl/HasPiece")
    private boolean hasPiece = false;

    public PieceControl(
            Intake intake,
            Indexer indexer,
            Elevator elevator,
            Ampper ampper,
            ShooterCmds shooterCmds,
            PicoColorSensor piPico) {
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        this.ampper = ampper;
        this.shooterCmds = shooterCmds;
        this.piPico = piPico;
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
    public Command shootWhenReady(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier, BooleanSupplier atDesiredAngle) {
        return
            Commands.waitUntil(() -> 
                    (PoseCalculations.inSpeakerShotZone(poseSupplier.get().getTranslation())
                        ? shooterCmds.shooterCalc.readyToShootSupplier().getAsBoolean() 
                        : shooterCmds.shooterCalc.readyToPassSupplier().getAsBoolean())
                    && atDesiredAngle.getAsBoolean()
                )
            .andThen(noteToShoot(poseSupplier, speedSupplier));
                
    }

    public Command shootPreload() {
        return Commands.waitUntil(shooterCmds.shooterCalc.readyToShootSupplier())
                .andThen(intakeAuto());
    }

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

    public Command noteToShootUsingSensorWhenReady(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        // this should be ran while we are aiming with pivot and shooter already
        // start running indexer so it gets up to speed and wait until shooter is at desired 
        // rotation and speed before sending note from ampper into indexer and then into 
        // shooter before stopping ampper and indexer
        return Commands.sequence(
                Commands.waitUntil(shooterCmds.shooterCalc.readyToShootSupplier()),
                noteToShootUsingSensor(poseSupplier, speedSupplier));
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
                Commands.waitUntil(() -> !piPico.hasNoteShooter() || FieldConstants.IS_SIMULATION),
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
            Commands.waitUntil(piPico::hasNoteShooter),
            stopIntakeAndIndexer(),
            indexer.toElevatorSlow(),
            ampper.outtakeSlow(),
            Commands.waitSeconds(.1),
            ampper.stopCommand(),
            Commands.waitSeconds(0.2),
            stopIntakeAndIndexer()
        );
    }

    public Command intakeForDoubleAmp() {
        return Commands.sequence(
            intake.inCommandSlow(.85),
            ampper.intakeSlow(),
            indexer.stopCommand(),
            Commands.runOnce(this::restartDoubleAmpTimer)
        );
    }

    public Command doubleAmpElevatorEnd() {
        return Commands.either(elevatorUpWhileIntaking(), elevatorToBottom(), this::doubleAmpTimerReady);
    }

    public Command elevatorUpWhileIntaking() {
        return elevator.toTopCommand().deadlineWith(intake.inCommandFor(.5)).andThen(intake.stopCommand());
    }

    public Command blepNote() {
        return Commands.parallel(
            intake.inCommand(),
            ampper.intake(),
            indexer.toShooter(),
            shooterCmds.setTripletCommand(new SpeedAngleTriplet(500, 500, 0))
        ).andThen(
            Commands.waitUntil(() -> !piPico.hasNoteShooter()),
            stopAllMotors()
        );
    }
    
    public Command intakeNoteDriver(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
        return Commands.either(
            noteToShoot(poseSupplier, speedSupplier)
                .andThen(
                    Commands.defer(
                        () -> intakeNoteDriver(poseSupplier, speedSupplier), 
                        intakeUntilNote().getRequirements()
                    )
                ),
            intakeUntilNote(), 
            piPico::hasNoteShooter);
    }

    public Command noteToIndexer() {
        return Commands.sequence(
            intake.inCommand(),
            ampper.intake(),
            indexer.toShooterSlow(),
            Commands.waitUntil(piPico::hasNoteShooter),
            stopIntakeAndIndexer()
        );
    }

    public Command noteToTrap() {
        return Commands.sequence(
            ampper.outtake(),
            shooterCmds.stowPivot(),
            indexer.toElevator(),   
            intake.inCommandSlow(),
            Commands.either(
                Commands.waitSeconds(.1),
                Commands.waitUntil(() -> !piPico.hasNoteShooter()),
                () -> FieldConstants.IS_SIMULATION),
            NT.getWaitCommand("noteToTrap1"), // 0.2
            stopIntakeAndIndexer(),
            ampper.outtakeSlow(),
            NT.getWaitCommand("noteToTrap2"), // 0.5
            stopIntakeAndIndexer()
        );
    }

    public Command noteToTrap2() {
        return Commands.sequence(
            ampper.setPercentCommand(-.7),
            shooterCmds.stowPivot(),
            indexer.setPercentCommand(.7),   
            intake.inCommandSlow(),
            Commands.either(
                Commands.waitSeconds(.1),
                Commands.waitUntil(() -> !piPico.hasNoteShooter()),
                () -> FieldConstants.IS_SIMULATION),
            Commands.waitSeconds(.1),
            Commands.either(
                Commands.waitSeconds(.1),
                Commands.waitUntil(() -> piPico.fallingEdgeHasNoteElevator()),
                () -> FieldConstants.IS_SIMULATION).withTimeout(.254), 
            stopIntakeAndIndexer()
        );
    }

    public Command noteToTrap3() {
        return Commands.either(noteToTrap2(), noteToTrap(), () -> piPico.shooterSensorConnected() && piPico.elevatorSensorConnected());
    }

    public Command ejectNote() {
        //outtakes the note
        return Commands.sequence(
            intake.outCommand(),
            shooterCmds.stowPivot(),
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
        return shooterCmds.stowPivot().andThen(ampper.intake().alongWith(indexer.toElevator()).alongWith(intake.outCommand()));
    }

    public Command stopPanicEject() {
        return ampper.stopCommand().alongWith(indexer.stopCommand()).alongWith(intake.stopCommand());
    }

    public Command intakeAuto() {
        return Commands.sequence(
                intake.inCommand(),
                ampper.intake(),
                indexer.toShooter());
    }

    public Command noteToTarget(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier, BooleanSupplier atDesiredAngle, BooleanSupplier operatorWantsToIntake) {
        // Defer the command to not require anyhing until it is actually ran
        // This is becuase placeTrap requires pivot, but shootWhenReady does not
        // If they are both required, then we would cancel any command that requires pivot
        // such as prepareSWDCommand
        return 
            new SelectiveConditionalCommand(
                Commands.race(
                    shootWhenReady(poseSupplier, speedSupplier, atDesiredAngle)
                        .andThen(setHasPiece(false)),
                    Commands.waitUntil(() -> elevator.getDesiredPosition() > ElevatorConstants.BOTTOM_POS)
                ),
                Commands.race(
                    placeWhenReady()
                        .andThen(setHasPiece(false)),
                    Commands.waitUntil(() -> elevator.getDesiredPosition() <= ElevatorConstants.BOTTOM_POS)
                ).andThen(ampper.stopCommand()),
                () -> elevator.getDesiredPosition() <= ElevatorConstants.BOTTOM_POS
            )
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                .raceWith(Commands.waitSeconds(4))
                .andThen(
                    Commands.defer(
                        () -> intakeUntilNote().onlyIf(operatorWantsToIntake), 
                        intakeUntilNote().getRequirements()));
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
        return setElevatorPosition(() -> ElevatorConstants.BOTTOM_POS)
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
                ampper.outtake(NT.getValue("placeOuttake"))
                    .andThen(
                        elevatorToBottom()
                        .alongWith(shooterCmds.raisePivot())
                        .alongWith(ampper.stopCommand())
                    ),
                setPlaceWhenReadyCommand(true),
                elevator::atDesiredPosition);
    }

    @AutoLogOutput (key = "Managers/PieceControl/PlaceWhenReady")
    private boolean placeWhenReady = false;

    public boolean shouldPlaceWhenReady() {
        return placeWhenReady;
    }

    public Command setPlaceWhenReadyCommand(boolean placeWhenReady) {
        return Commands.runOnce(() -> this.placeWhenReady = placeWhenReady);
    }

    public Command sourceShooterIntake() {
        return Commands.sequence(
            shooterCmds.sourceIntakeCommand(),
            indexer.toElevator(),
            ampper.outtake(),
            Commands.waitUntil(piPico::hasNoteShooter),
            Commands.waitSeconds(0.1),
            shooterCmds.stopShooter(),
            stopIntakeAndIndexer(),
            shooterCmds.stowPivot()
        );
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

    @AutoLogOutput (key = "Managers/PieceControl/DesiredSide")
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

    public Command moveNoteThenElevator() {
        return Commands.sequence(
            moveNote(false),
            elevator.toTopCommand(),
            prepPiece());
    }

    // Within a range of the [red circle](https://www.desmos.com/calculator/cu3ocssv5d)
    public Command getAutomaticShooterSpeeds(Supplier<Pose2d> robotPose, BooleanSupplier intaking, BooleanSupplier manualActivation) {
        return 
            new ActiveConditionalCommand(
                Commands.run(
                    () -> 
                        shooterCmds.setSpeeds(
                            PoseCalculations.inSpeakerShotZone(robotPose.get().getTranslation())
                                ? shooterCmds.shooterCalc.calculateSpeakerTriplet(
                                    robotPose.get().getTranslation()
                                ).getSpeeds()
                                : shooterCmds.shooterCalc.calculatePassTriplet(
                                    robotPose.get()
                                ).getSpeeds()
                        ),
                    shooterCmds.getShooter()
                ),
                shooterCmds.stopShooter(),
                () -> 
                    ((((piPico.hasNoteShooter() || piPico.hasNoteElevator()
                            && RobotContainer.distanceToSpeakerMeters < FieldConstants.AUTOMATIC_SHOOTER_DISTANCE_RADIUS_METERS)
                        || (RobotContainer.distanceToSpeakerMeters < FieldConstants.SPEAKER_CLEANUP_DISTANCE_METERS 
                            && intaking.getAsBoolean() 
                            && elevator.getDesiredPosition() < ElevatorConstants.NOTE_FIX_POS))

                        && (FieldConstants.IS_SIMULATION || piPico.shooterSensorConnected()))
                    
                    || (Robot.currentTimestamp - RobotContainer.gameModeStart < 7
                        && Robot.gameMode == GameMode.TELEOP 
                        && DriverStation.isFMSAttached())

                    || manualActivation.getAsBoolean()))
                .onlyIf(() -> Robot.gameMode != GameMode.TEST);
    }

    private Timer doubleAmpTimer = new Timer();
    public void restartDoubleAmpTimer() {
        doubleAmpTimer.restart();
    }

    public boolean doubleAmpTimerReady() {
        return doubleAmpTimer.get() > 0.35;
    }
}