package frc.robot.leds.commands.animations;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import java.util.Set;

// TODO: convert this to use wave logic with shortened stip length
public class LEDFlashCommand extends LEDCommands {
    private final Color color1;
    private final Color color2;
    private final double blinkTime;
    private final double blinkSpeed;
    private double startTime;
    private boolean isFinished;
    private LEDCommands root;

    public LEDFlashCommand(LEDCommands root, double blinkTime, double blinkSpeed, Color color1, Color color2) {
        this(root, blinkTime, blinkSpeed, color1, color2, Set.of(root.ledStrip));
    }

    public LEDFlashCommand(LEDCommands root, double blinkTime, double blinkSpeed, Color color1, Color color2, Set<Subsystem> requirements) {
        super(root.ledStrip, requirements);
        this.root = root;
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
        double currentTime = Robot.currentTimestamp - startTime;
        double blinkInterval = blinkTime / (blinkSpeed*blinkSpeed);
        root.ledStrip.setLED((currentTime % (2 * blinkInterval)) < blinkInterval ? color1 : color2);
        if (currentTime >= blinkTime) {
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        root.ledStrip.setLED(color1);
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}