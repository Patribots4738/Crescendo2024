package frc.robot.commands.handoff;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TriggerWheel;

public class Handoff {
    private final Command emptyCommand = Commands.runOnce(() -> System.out.println(""));
    private NotePosition notePosition;
    private TriggerWheel trigger;
    private Elevator elevator;
    private Intake intake;

    public Handoff() {
        intake = new Intake();
        trigger = new TriggerWheel();
        elevator = new Elevator();

        this.notePosition = trigger.getHasPiece().getAsBoolean() 
                                ? NotePosition.TRIGGER 
                                : NotePosition.NONE;
    }
    
    /**
     * @return
     */
    public NotePosition getNotePosition(){
        return this.notePosition;
    }

    /**
     * @param notePosition
     */
    public void setNotePosition(NotePosition notePosition) {
        this.notePosition = notePosition;
    }

    public void triggerReady() {
        trigger.setIsReady(intake.hasGamePieceTrigger());
    }

    public Trigger readyToShoot() {
        return new Trigger(trigger.getHasPiece());
    }

    public Command triggerToShooter() {
        return Commands.either(trigger.shootToPivot(), emptyCommand, this.readyToShoot());
    }

    public Command triggerToElevator() {
        return Commands.either(trigger.shootToElevator(), emptyCommand, this.readyToShoot());
    }

}
