package frc.robot.leds.commands.animations;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.leds.LedStrip;

public class LEDCommands extends Command {
    protected LedStrip ledStrip;    
    static double hueTime = 1;
    static double valueTime = 1;
    
    public LEDCommands(LedStrip ledStrip, Set<Subsystem> requirements) {
        super();
        super.addRequirements(ledStrip);
        super.addRequirements(requirements.toArray(new Subsystem[0]));
        this.ledStrip = ledStrip;
    }

    public LEDCommands(LedStrip ledStrip) {
        this(ledStrip, Set.of());
    }

    public enum HSV_Flash {
        HUE, SATURATION, VALUE, FLASH
    }

    protected boolean evalRoot(LEDCommands root) {
        return root == null;
    }
}
