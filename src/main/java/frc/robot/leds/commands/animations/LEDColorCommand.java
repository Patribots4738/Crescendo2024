package frc.robot.leds.commands.animations;

import edu.wpi.first.wpilibj.util.Color;

public class LEDColorCommand extends LEDCommands {
    private Color color;
    private int index = -1;
    private int index1 = 0;
    private int index2 = 10;
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

    public LEDColorCommand(LEDCommands root, Color color, int i, int j) {
      this(root, color);
      this.index1 = i;
      this.index2 = j;
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
