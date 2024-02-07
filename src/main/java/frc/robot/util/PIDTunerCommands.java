package frc.robot.util;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
public class PIDTunerCommands {
    
    private PIDNotConstants[] subsystems;
    private int subsystemIndex;
    private int PIDIndex;

    public PIDTunerCommands(PIDNotConstants[] subsystems) {
        this.subsystems = subsystems;
        this.subsystemIndex = 0;
        this.PIDIndex = 0;
    }

    public PIDNotConstants getCurrentSubsystem() {
        return this.subsystems[subsystemIndex];
    }

    public void increment() {
        if (subsystemIndex < subsystems.length - 1) {
            this.subsystemIndex++;
        }
    }

    public void PIDIncrement() {
        if (PIDIndex < 2) {
            this.PIDIndex++;
        }
    }

    public void PIDDecrease() {
        if (PIDIndex > 0) {
            this.PIDIndex--;
        }
    }

    public void log() {
        System.out.println("Subsystem: " + subsystemToString(subsystemIndex));
        System.out.println(subsystems[subsystemIndex].toString());
        System.out.println("PID Index: " + this.PIDIndex);
    }


    public String subsystemToString(int index) {
        switch (index) {
            case 0: 
                return "Pivot";
            case 1: 
                return "Shooter";
            case 2:
                return "Elevator";
            case 3:
                return "Climb";
        }
        return "";
    }

    public void decrease() {
        if (subsystemIndex > 0) {
            this.subsystemIndex--;
        }
    }
    public void increasePID(double value) {
        switch (this.PIDIndex) {
            case 0:
                subsystems[subsystemIndex].setP(subsystems[subsystemIndex].getP() + value);
                break;
            case 1: 
                subsystems[subsystemIndex].setI(subsystems[subsystemIndex].getI() + .01 * value);
                break;
            case 2: 
                subsystems[subsystemIndex].setD(subsystems[subsystemIndex].getD() + value);
                break;
        }
        log();
    }


    public Command incrementSubsystemCommand() {
        return Commands.runOnce(() -> increment());
    }
    public Command decreaseSubsystemCommand() {
        return Commands.runOnce(() -> decrease());
    }
    public Command increaseCurrentPIDCommand(double value) {
        return Commands.runOnce(() -> this.increasePID(value));
    }
    public Command decreaseCurrentPIDCommand(double value) {
        return Commands.runOnce(() -> this.increasePID(-value));
    }

    public Command PIDIncrementCommand() {
        return Commands.runOnce(() -> PIDIncrement());
    }
    public Command PIDDecreaseCommand() {
        return Commands.runOnce(() -> PIDDecrease());
    }
    public Command logCommand() {
        return Commands.runOnce(() -> log());
    }
}
