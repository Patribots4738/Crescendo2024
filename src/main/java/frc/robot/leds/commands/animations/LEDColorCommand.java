package frc.robot.leds.commands.animations;

import edu.wpi.first.wpilibj.util.Color;

public class LEDColorCommand extends LEDCommands {
    private Color color;
    private int index = -1;
    private LEDCommands root;

    public LEDColorCommand(LEDCommands root, Color color) {
        super(root.ledStrip);
        this.root = root;
        this.color = color;
    }

    public LEDColorCommand(LEDCommands root, Color color, int i) {
        this(root, color);
        this.index = i;
    }
    
    @Override
    public void execute() {
        root.ledStrip.setLED(index, color);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
