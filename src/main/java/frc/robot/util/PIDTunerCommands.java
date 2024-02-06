package frc.robot.util;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
public class PIDTunerCommands {
    
    private PIDNotConstants[] subsystems;
    private Command[] PIDChangeArray;
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
        if (subsystemIndex < subsystems.length) {
            this.subsystemIndex++;
        }
    }
    public void PIDIncrement() {
        if (PIDIndex < PIDChangeArray.length) {
            this.PIDIndex++;
        }
    }

    public void PIDDecrease() {
        if (PIDIndex > 0) {
            this.PIDIndex--;
        }
    }

    public void decrease() {
        if (subsystemIndex > 0) {
            this.subsystemIndex--;
        }
    }
    public void increasePID(double value) {
        switch (PIDIndex) {
            case 0:
                subsystems[subsystemIndex].setP(value);
            case 1: 
                subsystems[subsystemIndex].setI(.01 * value);
            case 2: 
                subsystems[subsystemIndex].setD(value);
        }
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
        return Commands.runOnce(() -> PIDIncrement());
    }
}
