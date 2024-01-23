package frc.robot.commands.handoff;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TriggerWheel;
import frc.robot.subsystems.ElevatorSubs.Claw;
import frc.robot.subsystems.ElevatorSubs.Elevator;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class Handoff {
    private final Command emptyCommand = Commands.runOnce(() -> System.out.println(""));
    private NotePosition currentNotePos;

    private Intake intake;
    private TriggerWheel trigger;
    
    private Elevator elevator;
    private Claw claw;

    private Pivot pivot;
    private Shooter shooter;


    public Handoff(
            Intake intake, 
            TriggerWheel trigger, 
            Elevator elevator, 
            Claw claw, 
            Pivot pivot, 
            Shooter shooter) 
        {
        this.intake = intake;
        this.trigger = trigger;
        this.elevator = elevator;
        this.claw = claw;
        this.pivot = pivot;
        this.shooter = shooter;

        this.currentNotePos = trigger.hasPiece().getAsBoolean() 
                                ? NotePosition.TRIGGER 
                                : NotePosition.NONE;
    }
    
    /**
     * @return
     */
    public NotePosition getCurrentNotePos(){
        return this.currentNotePos;
    }

    /**
     * @param notePosition
     */
    public void setCurrentNotePos(NotePosition notePosition) {
        this.currentNotePos = notePosition;
    }

    public void triggerReady() {
        trigger.setIsReady(intake.hasGamePieceTrigger());
    }

    public Trigger readyToShoot() {
        return new Trigger(trigger.hasPiece());
    }

    public Command triggerToShooter() {
        return Commands.either(trigger.toShooter(), emptyCommand, this.readyToShoot());
    }

    public Command triggerToElevator() {
        return Commands.either(trigger.toTrap(), emptyCommand, this.readyToShoot());
    }

}
