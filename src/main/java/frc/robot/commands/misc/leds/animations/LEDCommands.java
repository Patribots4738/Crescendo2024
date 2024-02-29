package frc.robot.commands.misc.leds.animations;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.misc.leds.LedStrip;

public class LEDCommands extends DeferredCommand {
    protected static LedStrip ledStrip;    
    static double waveHueOffset = 1;
    static double waveValueOffset = 1;
    
    public LEDCommands(LedStrip ledStrip, Set<Subsystem> requirements) {
        super(() -> Commands.none(), requirements);
        super.addRequirements(ledStrip);
        LEDCommands.ledStrip = ledStrip;
    }

    public LEDCommands(LedStrip ledStrip) {
        super(() -> Commands.none(), Set.of(ledStrip));
        LEDCommands.ledStrip = ledStrip;
    }

    public enum HSV {
        HUE, SATURATION, VALUE
    }
}
