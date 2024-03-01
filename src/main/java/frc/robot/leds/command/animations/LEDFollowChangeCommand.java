package frc.robot.leds.commands.animations;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

public class LEDFollowChangeCommand extends LEDCommands {
    private DoubleSupplier motorPose;
    private final double multiplier;
    private Color color;

    public LEDFollowChangeCommand(DoubleSupplier motorPosition, double multiplier, Color color) {
        super(ledStrip, Set.of());
        this.motorPose = motorPosition;
        this.color = color;
        this.multiplier = multiplier;
    }

    @Override
    public void execute() {
        double newMotorPose = motorPose.getAsDouble();

        newMotorPose = MathUtil.applyDeadband(newMotorPose * multiplier * ledStrip.getBuffer().getLength(), 0.01);

        for (int i = 0; i < ledStrip.getBuffer().getLength(); i++) {
            if (i < newMotorPose) {
                ledStrip.setLED(i, color);
            } else {
                ledStrip.setLED(i, Color.kBlack);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
