package frc.robot.commands.handoff;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ElevatorSubs.Claw;
import frc.robot.subsystems.ElevatorSubs.Elevator;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class Handoff {
    private final Command emptyCommand = Commands.runOnce(() -> System.out.println(""));
    private NotePosition notePosition = NotePosition.NONE;

    private Intake intake;
    private Indexer indexer;
    
    // TODO: Use the commands for the subsystems instead of the subsystems themselves
    private Elevator elevator;
    private Claw claw;

    //TODO: Use the commands for the subsystems instead of the subsystems themselves
    private Pivot pivot;
    private Shooter shooter;


    public Handoff(
            Intake intake, 
            Indexer indexer, 
            Elevator elevator, 
            Claw claw, 
            Pivot pivot, 
            Shooter shooter) 
        {
        this.intake = intake;
        this.indexer = indexer;
        this.elevator = elevator;
        this.claw = claw;
        this.pivot = pivot;
        this.shooter = shooter;
    }

    public Trigger readyToShoot() {
        return new Trigger(() -> pivot.atDesiredAngle() && shooter.atDesiredRPM());
    }

    // TODO: We assume that the note is already in the claw when we want to shoot
    public Command clawToShooter(){
        // this.notePosition = NotePosition.CLAW; ^^^
        Command shoot = emptyCommand;
        //TODO: impliment a way to tell the pivot and shooter to go to the correct angles etc... (before checking if ready to shoot)
        if ( this.readyToShoot().getAsBoolean() ) {
            // run the indexer and intake to make sure the note gets to the shooter
            shoot = 
                indexer.toShooter()
                    .andThen(intake.inCommand()) // TODO: is this correct? because the top and bottom motors are seperate
                    .andThen(claw.expel());

            this.notePosition = NotePosition.SHOOTER;
            //TODO: Then once we are done shooting, we want the variable to be set to NONE
        }

        return shoot;
    }

    public Command clawToTrap() {
        return emptyCommand;
    }

    // public void triggerReady() {
    //     trigger.setIsReady(intake.hasGamePieceTrigger());
    // }

    // public Trigger readyToShoot() {
    //     return new Trigger(trigger.hasPiece());
    // }

    // public Command triggerToShooter() {
    //     return Commands.either(trigger.toShooter(), emptyCommand, this.readyToShoot());
    // }

    // public Command triggerToElevator() {
    //     return Commands.either(trigger.toTrap(), emptyCommand, this.readyToShoot());
    // }

}
