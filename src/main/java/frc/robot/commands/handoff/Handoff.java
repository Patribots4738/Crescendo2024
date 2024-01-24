package frc.robot.commands.handoff;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ShooterCalc;
import frc.robot.subsystems.ElevatorSubs.Claw;
import frc.robot.subsystems.ElevatorSubs.Elevator;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Constants.TrapConstants;

public class Handoff {
    private final Command emptyCommand = null;
    private NotePosition notePosition = NotePosition.NONE;

    private Intake intake;
    private Indexer indexer;
    
    // TODO: Use the commands for the subsystems instead of the subsystems themselves
    private Elevator elevator;
    private Claw claw;

    //TODO: Use the commands for the subsystems instead of the subsystems themselves
    private Pivot pivot;
    private Shooter shooter;

    private ShooterCalc shooterCalc;

    private Swerve swerve;

    public Handoff(
            Intake intake, 
            Indexer indexer, 
            Elevator elevator, 
            Claw claw, 
            Pivot pivot, 
            Shooter shooter,
            ShooterCalc shooterCalc,
            Swerve swerve) 
        {
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        this.claw = claw;
        this.pivot = pivot;
        this.shooter = shooter;
        this.shooterCalc = shooterCalc;
        this.swerve = swerve;
    }

    public Trigger readyToShoot() {
        return new Trigger(() -> pivot.atDesiredAngle() && shooter.atDesiredRPM());
    }

    public Command stopAllMotors() {
        return Commands.parallel(
            intake.stop(),
            indexer.stop(),
            elevator.stop(),
            claw.stop(),
            shooter.stop()
        );
    }

    // TODO: We assume that the note is already in the claw when we want to shoot
    public Command clawToShooter(){
        // this.notePosition = NotePosition.CLAW; ^^^
        Command shoot = emptyCommand;
        this.shooterCalc.prepareFireMovingCommand(() -> true, swerve);
        if ( this.readyToShoot().getAsBoolean() ) {
            // run the indexer and intake to make sure the note gets to the shooter
            shoot = 
                indexer.toShooter()
                    .andThen(intake.inCommand()) // TODO: is this correct? because the top and bottom motors are seperate
                    .andThen(claw.expel())
                    .andThen(stopAllMotors());

            this.notePosition = NotePosition.SHOOTER;
            //TODO: Then once we are done shooting, we want the variable to be set to NONE
        }

        return shoot;
    }

    public Command clawToTrap() {
        Command shoot = emptyCommand;

        if ( this.notePosition == NotePosition.CLAW ) {
            // maybe make setPosition a command ORR Make the Elevator Command 
            shoot = Commands.runOnce(
                        () -> elevator.setPosition(TrapConstants.TRAP_POS)
            ).andThen(
                Commands.waitUntil(elevator.isAtTargetPosition())
            ).andThen(claw.expel())
            .andThen(new WaitCommand(1))
            .andThen(stopAllMotors());

            this.notePosition = NotePosition.CLAW;

        }
        return shoot;
    }

}
