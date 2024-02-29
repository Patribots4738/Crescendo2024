package frc.robot.commands.misc.leds.animations;

import edu.wpi.first.wpilibj.util.Color;

public class LEDColorCommand extends LEDCommands {
    private Color color;

    public LEDColorCommand(Color color) {
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
