// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc.leds;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.misc.leds.LedStrip;

public class LEDCommand extends DeferredCommand {
    protected static LedStrip ledStrip;    
    static double waveHueOffset = 1;
    static double waveValueOffset = 1;
    
    public LEDCommand(LedStrip ledStrip, Set<Subsystem> requirements) {
        super(() -> Commands.none(), requirements);
        super.addRequirements(ledStrip);
        LEDCommand.ledStrip = ledStrip;
    }

    public LEDCommand(LedStrip ledStrip) {
        super(() -> Commands.none(), Set.of(ledStrip));
        LEDCommand.ledStrip = ledStrip;
    }

    public enum HSV {
        HUE, SATURATION, VALUE
    }
}
