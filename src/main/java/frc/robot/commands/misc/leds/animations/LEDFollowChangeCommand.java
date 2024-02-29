package frc.robot.commands.misc.leds.animations;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.constants.Constants.TrapConstants;

public class LEDFollowChangeCommand extends LEDCommands {
    private DoubleSupplier motorPose;
    private final double multiplier;
    private Color color;

    public LEDFollowChangeCommand(DoubleSupplier elevatorMotorPosition, double multiplier, Color color) {
        super(ledStrip, Set.of());
        this.motorPose = elevatorMotorPosition;
        this.color = color;
        this.multiplier = multiplier;
    }

    @Override
    public void execute() {
        double elevatorPose = motorPose.getAsDouble();

        elevatorPose = MathUtil.applyDeadband(elevatorPose * multiplier * ledStrip.getBuffer().getLength(), 0.01);
        System.out.println("Elevator Pose: " + elevatorPose);

        for (int i = 0; i < ledStrip.getBuffer().getLength(); i++) {
            if (i < elevatorPose) {
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
