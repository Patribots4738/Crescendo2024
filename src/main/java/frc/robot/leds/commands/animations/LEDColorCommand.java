package frc.robot.leds.commands.animations;

import edu.wpi.first.wpilibj.util.Color;

public class LEDColorCommand extends LEDCommands {
    private Color color;
    private int index1 = -1;
    private int index2 = -1;
    private LEDCommands root;

    public LEDColorCommand(LEDCommands root, Color color) {
        super(root.ledStrip);
        this.root = root;
        this.color = color;
    }

    public LEDColorCommand(LEDCommands root, Color color, int index) {
        this(root, color);
        this.index1 = index;
        this.index2 = index + 1;
    }

    public LEDColorCommand(LEDCommands root, Color color, int index1, int index2) {
        this(root, color);
        this.index1 = index1;
        this.index2 = index2;
    }

    @Override
    public void execute() {
        root.ledStrip.setLED(index1, index2, color);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        root.ledStrip.setLED(index1, index2, Color.kBlack);
    }
}
