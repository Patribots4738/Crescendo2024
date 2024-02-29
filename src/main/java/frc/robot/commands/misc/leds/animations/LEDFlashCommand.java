package frc.robot.commands.misc.leds.animations;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import java.util.Set;

public class LEDFlashCommand extends LEDCommands {
    private final Color color1;
    private final Color color2;
    private final double blinkTime;
    private final double blinkSpeed;
    private double startTime;
    private boolean isFinished;

    public LEDFlashCommand(double blinkTime, double blinkSpeed, Color color1, Color color2) {
        this(blinkTime, blinkSpeed, color1, color2, Set.of(ledStrip));
    }

    public LEDFlashCommand(double blinkTime, double blinkSpeed, Color color1, Color color2, Set<Subsystem> requirements) {
        super(ledStrip, requirements);
        this.color1 = color1;
        this.color2 = color2;
        this.blinkTime = blinkTime;
        this.blinkSpeed = blinkSpeed;
        this.isFinished = false;
    }

    @Override
    public void initialize() {
        startTime = Robot.currentTimestamp;
    }

    @Override
    public void execute() {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime;
        double blinkInterval = blinkTime / (blinkSpeed*blinkSpeed);
        for (int i = 0; i < ledStrip.getBuffer().getLength(); i++) {
            ledStrip.setLED(i, (currentTime % (2 * blinkInterval)) < blinkInterval ? color1 : color2);
        }
        if (currentTime >= blinkTime) {
            isFinished = true;
            for (int i = 0; i < ledStrip.getBuffer().getLength(); i++) {
                ledStrip.setLED(i, color2);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}