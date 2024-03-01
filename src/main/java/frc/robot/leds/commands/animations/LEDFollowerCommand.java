package frc.robot.leds.commands.animations;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

public class LEDFollowerCommand extends LEDCommands {
    private DoubleSupplier motorPoseSupplier;
    private final double multiplier;
    private Color color;
    private LEDCommands root;

    public LEDFollowerCommand(LEDCommands root, DoubleSupplier motorPosition, double multiplier, Color color) {
        super(root.ledStrip, Set.of());
        this.motorPoseSupplier = motorPosition;
        this.color = color;
        this.multiplier = multiplier;
        this.root = root;
    }

    @Override
    public void execute() {
        double motorPose = motorPoseSupplier.getAsDouble();

        motorPose = MathUtil.applyDeadband(motorPose * multiplier * root.ledStrip.getBuffer().getLength(), 0.01);
        
        for (int i = 0; i < root.ledStrip.getBuffer().getLength(); i++) {
            root.ledStrip.setLED(i, (i < motorPose) ? color : Color.kBlack);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
