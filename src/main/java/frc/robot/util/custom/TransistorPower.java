package frc.robot.util.custom;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TransistorPower extends AnalogOutput {
    public TransistorPower(int port) {
        super(port);
    }

    public TransistorPower(int port, boolean state) {
        super(port);
        set(state);
    }

    public void setState(boolean state) {
        setVoltage(state ? 5 : 0);
    }

    public void set(boolean state) {
        setState(state);
    }

    public boolean get() {
        return getVoltage() > 0.5;
    }

    public Command getPowerCycleCommand() {
        return Commands.runOnce(() -> set(false))
                    .andThen(Commands.waitSeconds(1), 
                    Commands.runOnce(() -> set(true))).ignoringDisable(true);
    }

    public void schedulePowerCycleCommand() {
        getPowerCycleCommand().schedule();
    }
}
