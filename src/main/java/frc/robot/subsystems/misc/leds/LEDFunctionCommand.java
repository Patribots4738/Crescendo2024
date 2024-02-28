// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.misc.leds;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class LEDFunctionCommand extends DeferredCommand {
    protected static LedStrip ledStrip;

    public LEDFunctionCommand(LedStrip ledStrip, Set<Subsystem> requirements) {
        super(() -> Commands.none(), requirements);
        super.addRequirements(ledStrip);
        LEDFunctionCommand.ledStrip = ledStrip;
    }

    protected LEDFunctionCommand(LedStrip ledStrip) {
        super(() -> Commands.none(), Set.of(ledStrip));
        LEDFunctionCommand.ledStrip = ledStrip;
    }

    public enum HSV {
        HUE, SATURATION, VALUE
    }
}
