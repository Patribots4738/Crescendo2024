// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.leds.commands.animations;

import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BooleanSupplier;


public class LEDConditionalCommand extends LEDCommands {
    private final Color color1;
    private final Color color2;
    private final BooleanSupplier condition;
    private int index = -1;

    public LEDConditionalCommand(Color color1, Color color2, BooleanSupplier condition) {
        super(ledStrip);
        this.color1 = color1;
        this.color2 = color2;
        this.condition = condition;
    }

    public LEDConditionalCommand(Color color1, Color color2, BooleanSupplier condition, int i) {
        this(color1, color2, condition);
        this.index = i;
    }

    @Override
    public void execute() {
        if (condition.getAsBoolean()) {
            ledStrip.setLED(index, color1);
        } else {
            ledStrip.setLED(index, color2);
        }
    }
}
