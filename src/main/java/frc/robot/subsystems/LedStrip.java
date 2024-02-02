package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.google.gson.JsonElement;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.LEDConstants;

public class LedStrip extends SubsystemBase {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int currentPatternIndex;
    private final Supplier<Pose2d> poseSupplier;

    public final HashMap<Integer, Command> patternMap = new HashMap<>();

    public LedStrip(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        this.led = new AddressableLED(LEDConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();

        configureLED();

        patternMap.put(0, turnOff());
        patternMap.put(1, greenNGold());
        patternMap.put(2, circus());
        patternMap.put(3, loading());
        patternMap.put(5, alliance(() -> DriverStation.getAlliance().equals(Optional.of(Alliance.Red))));
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
            case (5) -> alliance(() -> DriverStation.getAlliance().equals(Optional.of(Alliance.Red)));
            case (6) -> flash();
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
    public Command turnOff() {
        return runOnce(() -> setLED(Color.kBlack));
    }

    private int rainbowOffset = 0;
    private Command rainbow() {
        return runOnce(() -> {
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
}
