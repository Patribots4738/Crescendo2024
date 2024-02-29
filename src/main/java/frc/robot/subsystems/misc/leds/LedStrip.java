package frc.robot.subsystems.misc.leds;

//import frc.robot.DriverUI;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.misc.leds.animations.FlashCommand;
import frc.robot.commands.misc.leds.animations.LEDCommands;
import frc.robot.commands.misc.leds.animations.WaveCommand;
import frc.robot.util.constants.Constants.LEDConstants;
import java.util.List;

public class LedStrip extends SubsystemBase {

	private AddressableLED led;
    public AddressableLEDBuffer ledBuffer;

    private int currentPatternIndex;
    //private final Supplier<Pose2d> poseSupplier;

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
            case (0) -> turnOff();
            case (1) -> greenNGold();
            case (2) -> circus();
            case (3) -> loading();
            case (5) -> alliance(Robot::isRedAlliance);
            case (6) -> new FlashCommand(30, 5.5, Color.kGreen, Color.kBlack);
            case (7) -> rainbow();
            default -> turnOff();
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

    public Command turnOff() {
        return runOnce(() -> setLED(Color.kBlack));
    }

    public Command rainbow() {
        return new WaveCommand().new Hue(
            180, 
            (double) ledBuffer.getLength(), 
            180, 
            0.05, 
            Math.PI*2);
    }

    double greenNGoldOffset = 1;

    public Command greenNGold() {
        return new WaveCommand().new Hue(
            50, 
            (double) ledBuffer.getLength(), 
            75, 
            .075, 
            Math.PI*2);
    }

    public Command circus() {
        return run(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final Color color = (((int) ((i/5.0) + greenNGoldOffset) % 2 == 0) ? Color.kRed : Color.kWhite);
                setLED(i, color);
            }
            greenNGoldOffset += .05;
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
        return new WaveCommand().new Saturation(
            340, 
            65, 
            255, 
            .025, 
            6.28, 
            isRedAlliance.getAsBoolean() ? 180 : 117);
    }

    
    Color Blue = new Color(0, 0, 255);
    Color Orange = new Color(255, 80, 0);
    Color GreerYeller = new Color(213, 255, 0);
    Color Greer = new Color(0, 250, 0);
    Color TurnOff = new Color(0, 0, 0);
    Color Red = new Color(250, 0, 0);
    Color NoColor = new Color(0, 0, 0);

    Color GradBlue = Color.fromHSV(70, 255, 255);
    Color GradRed = Color.fromHSV(0, 255, 255);

    public Command lightsoffLED() {
        return Commands.runOnce(() -> {
            setLED(Color.kBlack);
        });
    }

    public Command cautionLED() {
        return run(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final Color color = (i % 2 == 0) ? Color.kYellow : Color.kDarkGray;
                setLED(i, color);
            }
        });
    }

    public Command yellowgreerLED() {
        return Commands.runOnce(() -> {
            setLED(Color.kGreenYellow);
        });
    }

    public Command redLED() {
        return Commands.runOnce(() -> {
            setLED(Color.kRed);
        });
    }

    public Command almmondLED() {
        return Commands.run(() -> {
            setLED(Color.kBlanchedAlmond);
        });
    }

    public Command blueLED() {
        return Commands.run(() -> {
            setLED(Color.kBlue);
        });
    }

    public Command greerLED() {
        return Commands.runOnce(() -> {
            setLED(Color.kGreen);
        });
    }

}