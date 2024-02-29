package frc.robot.commands.misc.leds.animations;

import edu.wpi.first.wpilibj.util.Color;

public class LEDColorCommand extends LEDCommands {
    private Color color;
    private int index = -1;

    public LEDColorCommand(Color color) {
        super(ledStrip);
        this.color = color;
    }

    public LEDColorCommand(Color color, int i) {
        this(color);
        this.index = i;
    }
    
    @Override
    public void execute() {
        if (index == -1) {
            ledStrip.setLED(color);
        } else {
            ledStrip.setLED(index, color);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
