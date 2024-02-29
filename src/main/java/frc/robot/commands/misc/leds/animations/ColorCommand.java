package frc.robot.commands.misc.leds.animations;

import edu.wpi.first.wpilibj.util.Color;

public class ColorCommand extends LEDCommands {
    private Color color;

    public ColorCommand(Color color) {
        super(ledStrip);
        this.color = color;
    }
    
    @Override
    public void execute() {
        ledStrip.setLED(color);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
