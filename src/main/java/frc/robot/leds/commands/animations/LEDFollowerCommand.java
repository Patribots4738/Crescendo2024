package frc.robot.leds.commands.animations;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

public class LEDFollowerCommand extends LEDCommands {
    private DoubleSupplier motorPoseSupplier;
    private final double multiplier;
    private Color color;

    public LEDFollowerCommand(DoubleSupplier motorPosition, double multiplier, Color color) {
        super(ledStrip, Set.of());
        this.motorPoseSupplier = motorPosition;
        this.color = color;
        this.multiplier = multiplier;
    }

    @Override
    public void execute() {
        double motorPose = motorPoseSupplier.getAsDouble();

        System.out.println("newMotorPose: " + motorPose + " multiplier: " + multiplier + " ledStrip.getBuffer().getLength(): " + ledStrip.getBuffer().getLength());
        motorPose = MathUtil.applyDeadband(motorPose * multiplier * ledStrip.getBuffer().getLength(), 0.01);
        
        for (int i = 0; i < ledStrip.getBuffer().getLength(); i++) {
            ledStrip.setLED(i, (i < motorPose) ? color : Color.kBlack);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
