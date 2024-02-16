package frc.robot.subsystems;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.DriverUI;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.LEDConstants;

public class LedStrip extends SubsystemBase {

	private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int currentPatternIndex;
    private final Supplier<Pose2d> poseSupplier;

    public final HashMap<Integer, Command> patternMap = new HashMap<>();

    //public LedStrip() {
    //    poseSupplier = () -> new Pose2d();
    //}

    public LedStrip(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        this.led = new AddressableLED(LEDConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.PWM_PORT);
        LEDConstants.LED_COUNT = ledBuffer.getLength();

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();

        configureLED();

        patternMap.put(0, turnOff());
        patternMap.put(1, greenNGold());
        patternMap.put(2, circus());
        patternMap.put(3, loading());
        patternMap.put(5, alliance(() -> FieldConstants.IS_RED_ALLIANCE()));
        patternMap.put(6, flash());
    }

    public AddressableLEDBuffer getBuffer() {
        return ledBuffer;
    }
    
    @Override
    public void periodic() {
        runPattern(currentPatternIndex).schedule();
    }

    private Command runPattern(int index) {
        Command selectedPattern = switch (currentPatternIndex) {
            case (0) -> turnOff();
            case (1) -> greenNGold();
            case (2) -> circus();
            case (3) -> loading();
            case (5) -> alliance(() -> FieldConstants.IS_RED_ALLIANCE());
            case (6) -> flash();
            case (7) -> rainbow();
            default -> runOnce(() -> {
            });
        };
        return selectedPattern;
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

    private Command configureLED() {
        return runOnce(() -> {
        });
    }

    public void setLED(int startIndex, int endIndex, Color color) {
        for (int i = startIndex; i < endIndex; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void setLED(int index, Color color) {
        ledBuffer.setLED(index, color);
    }

    public void setLED(Pair<Integer, Integer> range, Color color) {
        setLED(range.getFirst(), range.getSecond(), color);
    }

    public void setLED(Color color) {
        setLED(0, ledBuffer.getLength(), color);
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
            double ratio = (i - startIndex) / (endIndex - startIndex);
            ledBuffer.setLED(i, sampleGradient(firstColor, secondColor, ratio));
        }
    }

    public void oohShiny(Color blinkI, Color blinkII, int blinkAmount, int setIndexI, int setIndexII) {
        for (int j = blinkAmount; j > 0; j--) {
            Color blinkColor = (j % 2 == 0) ? blinkI : blinkII;
            setLED(setIndexI, setIndexII, blinkColor);
        }
    }

    public Command turnOff() {
        return runOnce(() -> setLED(Color.kBlack));
    }

    private int rainbowOffset = 0;
    public Command rainbow() {
        return run(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int hue = (this.rainbowOffset + (i * 180 / ledBuffer.getLength())) % 180;
                setLED(i, Color.fromHSV(hue / 2, 255, 255));
            }
            this.rainbowOffset += 3;
            this.rainbowOffset %= 180;
        });
    }

    private int greenNGoldOffset = 0;
    public Command greenNGold() {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final int hue = 135 + (int) (15 * Math.sin(Math.PI * ((i + this.greenNGoldOffset * ledBuffer.getLength()) / ledBuffer.getLength())));
                setLED(i, Color.fromHSV(hue/2, 255, 255));
            }
            this.greenNGoldOffset += 1;
            this.greenNGoldOffset %= 10;
        });
    }

    public Command circus() {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final Color color = (i % 2 == 0) ? Color.kFirstRed : Color.kWhite;
                setLED(i, color);
            }
        });
    }

    public Command loading() {
        return runOnce(() -> {
            for (int i = 0; i < 2 && i > 8; i++) {
                final Color color = (i < 2 && i > 8) ? Color.kFirstBlue : Color.kBlack;
                setLED(i, color);
            }
        });
    }

    public Command flash() {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final Color color = (i % 2 == 0) ? Color.kOrange : Color.kBlack;
                setLED(i, color);
            }
        });
    }

    private int allianceOffset = 0;
    public Command alliance(BooleanSupplier isRedAlliance) {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final int hue = 135 + (int) 
                    (15 * 
                        Math.sin(
                            Math.PI 
                            * ((i + this.allianceOffset 
                                * ledBuffer.getLength()) 
                            / ledBuffer.getLength())
                        )
                    );
                setLED(i, Color.fromHSV(hue/2, 255, 255));
            }
            this.allianceOffset += 1;
            this.allianceOffset %= 10;
        });
    }
    
    Color Orange = new Color(250, 160, 30);
    Color GreerYeller = new Color(50, 250, 0);
    Color TurnOff = new Color(0, 0, 0);

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
        return Commands.runOnce(() -> {
            oohShiny(GreerYeller, TurnOff, 10, 1, 10);
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
            setLED(Color.kBlanchedAlmond);});
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

            Color desiredColor = Color.kAliceBlue;

            if (topmostActiveLED == LEDConstants.ELEVATOR_LED_COUNT) {
                desiredColor = ((int) (DriverUI.currentTimestamp * 5)) % 2 == 0
                    ? Color.kBlack 
                    : Color.kGreenYellow;
            }
            

            
            setLED(
                LEDConstants.ELEVATOR_LEFT_START_INDEX, 
                LEDConstants.ELEVATOR_LEFT_START_INDEX + topmostActiveLED, 
                desiredColor
            );
            setLED(
                LEDConstants.ELEVATOR_RIGHT_START_INDEX, 
                LEDConstants.ELEVATOR_RIGHT_START_INDEX + topmostActiveLED, 
                desiredColor
            );
            
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
