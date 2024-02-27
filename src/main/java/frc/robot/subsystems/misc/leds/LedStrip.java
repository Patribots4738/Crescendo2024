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
import frc.robot.util.constants.Constants.LEDConstants;
import java.util.List;
import java.util.ArrayList;

public class LedStrip extends SubsystemBase {

	private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int currentPatternIndex;
    //private final Supplier<Pose2d> poseSupplier;

    public final HashMap<Integer, Command> patternMap = new HashMap<>();

    public LedStrip() {
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
        runPattern(currentPatternIndex).schedule();
        led.setData(ledBuffer);
    }

    public Command changeLEDsPattern() {
        return Commands.runOnce(() -> {
            selectedLED += 1;
            selectedLED %= 9;
        });
    }

    public int selectedLED = 5;

    private Command runPattern(int index) {
        Command selectedPattern = switch (selectedLED) {
            case (0) -> turnOff();
            case (1) -> greenNGold();
            case (2) -> circus();
            case (3) -> loading();
            case (5) -> alliance(Robot::isRedAlliance);
            case (6) -> flash();
            case (7) -> rainbow();
            case (8) -> elevatorTOPLED();
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

    /**
     * 
     * firstColor is the color that the gradient starts on
     * @param firstColor
     * 
     * second Color is the color that the gradient ends on
     * @param secondColor
     * 
     * startIndex is the LED that the gradient starts on
     * @param startIndex
     * 
     * endIndex is the LED that the gradient ends on
     * @param endIndex
     */
    public void setLEDGradient(Color firstColor, Color secondColor, int startIndex, int endIndex) {
        for (int i = startIndex; i < endIndex; i++) {
            double ratio = (double) (i - startIndex) / (double) (endIndex - startIndex);
            setLED(i, sampleGradient(firstColor, secondColor, ratio));
        }
    }

    public void oohShiny(Color blinkI, Color blinkII, int blinkAmount, int setIndexI, int setIndexII) {
        System.out.println("Hello");
        for (int j = setIndexI; j < setIndexII; j++) {
            Color blinkColor = (j % 2 == 0) ? blinkI : blinkII;
            setLED(setIndexI, blinkColor);
        }
        
    }

    public Command turnOff() {
        return runOnce(() -> setLED(Color.kBlack));
    }

    private double rainbowOffset = 0;

    public Command rainbow() {
        return run(() -> {
            waveHueFunction(
                50, 
                (double) ledBuffer.getLength(), 
                2, 
                2,
                180);
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int hue = (int) (this.rainbowOffset + (i * 180 / ledBuffer.getLength())) % 180;
                setLED(i, Color.fromHSV(hue, 255, 255));
            }
            this.rainbowOffset += 2;
            this.rainbowOffset %= 180;
        });
    }

    double greenNGoldOffset = 1;

    public Command greenNGold() {
        return run(() -> {
            waveHueFunction(
                50,
                (double) ledBuffer.getLength(), 
                75,
                .075, 
                Math.PI*2);
        });
    }

    private void evaluateWave(double scale, double spacing, double time, int i, boolean isHue, double waveOffset) {
        double wave = Math.sin((10*i + (spacing * time)) / spacing);
        int output = (int) ((waveOffset + scale*wave) / (isHue ? 2 : 1));
        setLED(i, isHue ? Color.fromHSV(output, 255, 255) : Color.fromHSV(180, 255, output));
    }

    private double waveHueOffset = 1;
    public void waveHueFunction(double scale, double spacing, double waveOffset, double waveSpeed, double waveRange) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            evaluateWave(
                scale,
                spacing, 
                waveHueOffset,
                i, 
                true, 
                waveOffset
            );
        }
        waveHueOffset += waveSpeed;
        waveHueOffset %= waveRange;
    }
    public void resetWaveHueOffset() {
        waveHueOffset = 1;
    }

    private double waveValueOffset = 1;
    public void waveValueFunction(double scale, double spacing, double waveOffset, double waveValueIncrement, double waveRange) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            evaluateWave(
                scale,
                spacing, 
                waveOffset, 
                i, 
                false, 
                waveValueIncrement
            );
        }
        waveValueOffset += waveValueIncrement;
        waveValueOffset %= waveRange;
    }
    public void resetWaveValueOffset() {
        waveValueOffset = 1;
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

    public Command flash() {
        return run(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final Color color = (i + greenNGoldOffset % 2 == 0) ? Color.kOrange : Color.kBlack;
                setLED(i, color);
            }
            greenNGoldOffset++;
        });
    }

    private double allianceOffset = 0;


    // https://www.desmos.com/calculator/kxx5hbhyy1
    public Command alliance(BooleanSupplier isRedAlliance) {
        return run(() -> {
            boolean isRed = isRedAlliance.getAsBoolean();
            waveValueFunction(500, 50, 0, .025, 6.28);
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int saturation = MathUtil.clamp((int) (500*Math.abs(Math.sin((i+50.0*allianceOffset)/50.0))),0,255);
                int hue = isRed ? 0 : 227;
                setLED(i, Color.fromHSV(hue/2, saturation, 255));
            }
            this.allianceOffset += .025;
            this.allianceOffset %= 6.28;
        });
    }
    
    Color Orange = new Color(250, 160, 30);
    Color GreerYeller = new Color(50, 250, 0);
    Color Greer = new Color(0, 250, 0);
    Color TurnOff = new Color(0, 0, 0);
    Color Red = new Color(250, 0, 0);

    public Command upwardsElevatorGradientLED() {
        return Commands.run(() -> {
            setLEDGradient(Orange, GreerYeller, 1, 10);
        });
    }

    public Command backwardsElevatorGradientLED() {
        return Commands.run(() -> {
            setLEDGradient(GreerYeller, Orange, 10, 1);
        });
    }

    public Command elevatorTOPLED() {
        return Commands.run(() -> {
            oohShiny(GreerYeller, TurnOff, 999999999, 0, ledBuffer.getLength());
        }).ignoringDisable(true);
    }

    public Command cautionCoolDownLED() {
        return Commands.runOnce(() -> {
            oohShiny(Red, TurnOff, 999, 10, 1);
        });
    }

    public Command roboRiseLED() {
        return Commands.run(() -> {
            setLEDGradient(Greer, GreerYeller, 1, 100);
        });
    }

    public Command roboLowerLED() {
        return Commands.run(() -> {
            setLEDGradient(GreerYeller, Greer, 100, 1);
        });
    }

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

    public Command trackElevator(DoubleSupplier elevatorPositionSupplier) {
        return Commands.run(() -> {
            double currentElevatorPosition = elevatorPositionSupplier.getAsDouble();
            // convert the elevator position [0,1] 
            // to an LED index (int minIndex, int maxIndex)
            // did someone say interpolation???
            int topmostActiveLED = (int) Math.round(
                MathUtil.interpolate(
                    0, 
                    LEDConstants.ELEVATOR_LED_COUNT, 
                    currentElevatorPosition
                ));

        }).until(() -> {
            return elevatorPositionSupplier.getAsDouble() == 0;
        });
    }

    public Command fullElevatorCommand(DoubleSupplier elevatorPositionSupplier) {
        return Commands.sequence(
            Commands.run(() -> {})
                .until(() -> elevatorPositionSupplier.getAsDouble() == 1),
            Commands.run(()-> {})
                .until(() -> elevatorPositionSupplier.getAsDouble() < 0.95),
            Commands.run(()-> {})
                .until(() -> elevatorPositionSupplier.getAsDouble() == 0)
        );
    }

    private Color sampleGradient(Color firstColor, Color secondColor, double ratio) {
        return new Color(
            firstColor.red * ratio + secondColor.red * (1 - ratio), 
            firstColor.green * ratio + secondColor.green * (1 - ratio), 
            firstColor.blue * ratio + secondColor.blue * (1 - ratio));
    }
}