// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.leds.commands.animations;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConditionalCommand extends LEDCommands {
    private final Color color1;
    private final Color color2;
    private final boolean condition;
    private int index = -1;

    public LEDConditionalCommand(Color color1, Color color2, boolean condition) {
        super(ledStrip);
        this.color1 = color1;
        this.color2 = color2;
        this.condition = condition;
    }

    public LEDConditionalCommand(Color color1, Color color2, boolean condition, int i) {
        this(color1, color2, condition);
        this.index = i;
    }

    @Override
    public void execute() {
        if (condition) {
            new LEDColorCommand(color1, index).execute();
        } else {
            new LEDColorCommand(color2, index).execute();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
