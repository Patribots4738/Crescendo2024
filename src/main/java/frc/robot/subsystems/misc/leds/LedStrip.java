package frc.robot.subsystems.misc.leds;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.misc.leds.animations.LEDColorCommand;
import frc.robot.commands.misc.leds.animations.LEDFlashCommand;
import frc.robot.commands.misc.leds.animations.LEDCommands;
import frc.robot.commands.misc.leds.animations.LEDConditionalCommand;
import frc.robot.commands.misc.leds.animations.LEDWaveCommand;
import frc.robot.util.Constants.LEDConstants;

import java.util.List;

public class LedStrip extends SubsystemBase {

	private AddressableLED led;
    public AddressableLEDBuffer ledBuffer;

    private int currentPatternIndex;

    public final HashMap<Integer, Command> patternMap = new HashMap<>();

    private LEDCommands ledFunctionCommand = new LEDCommands(this);

    public LedStrip(DoubleSupplier elevatorPositionSupplier) {
        this.led = new AddressableLED(LEDConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        LEDConstants.LED_COUNT = ledBuffer.getLength();

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();
        setLED(Color.kBlack);
        led.setData(ledBuffer);
    }

    public AddressableLEDBuffer getBuffer() {
        return ledBuffer;
    }
    
    @Override
    public void periodic() {
        // runPattern(currentPatternIndex).execute();
        led.setData(ledBuffer);
    }

    public Command changeLEDsPattern() {
        return Commands.runOnce(() -> {
            selectedLED += 1;
            selectedLED %= 9;
        });
    }

    public int selectedLED = 7;

    private Command runPattern(int index) {
        Command selectedPattern = switch (selectedLED) {
            case (0) -> new LEDColorCommand(Color.kBlack);
            case (1) -> greenNGold();
            case (2) -> circus();
            case (3) -> loading();
            case (5) -> alliance(Robot::isRedAlliance);
            case (6) -> new LEDFlashCommand(30, 5.5, Color.kGreen, Color.kBlack);
            case (7) -> rainbow();
            default -> new LEDColorCommand(Color.kBlack);
        };
        return selectedPattern.ignoringDisable(true);
    }

    public Command setCurrentPattern(int index) {
        return runOnce(() -> {
            this.currentPatternIndex = index;
        });
    }

    public Command incrementPatternIndex() {
        return runOnce(() -> {
            currentPatternIndex++;
            currentPatternIndex %= patternMap.size();
        });
    }

    public IntSupplier getCurrentPatternSupplier() {
        return () -> currentPatternIndex;
    }

    public void setLED(int startIndex, int endIndex, Color color) {
        for (int i = startIndex; i < endIndex; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void setLED(int index, Color color) {
        setLED(index, index+1, color);
    }

    public void setLED(Pair<Integer, Integer> range, Color color) {
        setLED(range.getFirst(), range.getSecond(), color);
    }

    public void setLED(Color color) {
        setLED(0, ledBuffer.getLength(), color);
    }

    public void setLED(List<Color> colors) {
        for (Color color : colors) {
            setLED(0, ledBuffer.getLength(), color);
        }
    }

    public Command rainbow() {
        return new LEDWaveCommand().new Hue(
            180, 
            (double) ledBuffer.getLength(), 
            180, 
            0.05, 
            Math.PI*2);
    }

    
    public Command greenNGold() {
        return new LEDWaveCommand().new Hue(
            50, 
            (double) ledBuffer.getLength(), 
            75, 
            .075, 
            Math.PI*2);
    }
        
    double circusOffset = 1;
    public Command circus() {
        return run(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final Color color = (((int) ((i/5.0) + circusOffset) % 2 == 0) ? Color.kRed : Color.kWhite);
                setLED(i, color);
            }
            circusOffset += .05;
        });
    }

    public Command loading() {
        return run(() -> {
            for (int i = 0; i < 2 && i > 8; i++) {
                final Color color = (i < 2 && i > 8) ? Color.kFirstBlue : Color.kBlack;
                setLED(i, color);
            }
        });
    }

    // https://www.desmos.com/calculator/kuthptwuki
    public Command alliance(BooleanSupplier isRedAlliance) {
        return new LEDWaveCommand().new Saturation(
            340, 
            65, 
            255, 
            .025, 
            6.28, 
            isRedAlliance.getAsBoolean() ? 180 : 117);
    }

    public Command cautionLED() {
        return run(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                new LEDConditionalCommand(Color.kYellow, Color.kDarkGray, (i % 2 == 0), i).execute();
            }
        });
    }

}