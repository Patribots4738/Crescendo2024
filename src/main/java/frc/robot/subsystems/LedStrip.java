package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        patternMap.put(0, Commands.runOnce(() -> turnOff()));
        patternMap.put(1, Commands.runOnce(() -> greenNGold()));
        patternMap.put(2, Commands.runOnce(() -> circus()));
        patternMap.put(3, Commands.runOnce(() -> loading()));
        patternMap.put(4, Commands.runOnce(() -> LPI(poseSupplier.get())));
        patternMap.put(5,
                Commands.runOnce(() -> alliance(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))));
        patternMap.put(6, Commands.runOnce(() -> flash()));
    }

    private Command runPattern(int index) {
        Command selectedPattern = 
            switch (currentPatternIndex) {
                case (0) -> turnOff();
                case (1) -> greenNGold();
                case (2) -> circus();
                case (3) -> loading();
                case (4) -> LPI(poseSupplier.get());
                case (5) -> alliance(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)));
                case (6) -> flash();
                default -> runOnce(() -> {});
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

    public IntSupplier getCurrentPattern() {
        return () -> currentPatternIndex;
    }

    private Command configureLED() {
        return runOnce(() -> {});
    }

    public Command turnOff() {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setHSV(i, 0, 0, 0);
            }
        });
    }

    private int rainbowOffset;
    private Command rainbow() {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int hue = (this.rainbowOffset + (i * 180 / ledBuffer.getLength())) % 180;
                ledBuffer.setHSV(i, hue, 50, 100);
            }
            this.rainbowOffset += 3;
            this.rainbowOffset %= 180;
        });
    }

    // https://www.desmos.com/calculator/s5ylo1v9lt
    private int greenNGoldOffset = 0;

    public Command greenNGold() {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final int hue = 135 + (int) (15 * Math
                        .sin(Math.PI * ((i + this.greenNGoldOffset * ledBuffer.getLength()) / ledBuffer.getLength())));
                ledBuffer.setHSV(i, hue, 255, 255);
            }
            this.greenNGoldOffset += 1;
            this.greenNGoldOffset %= 10;
        });
    }

    public Command circus() {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final int hue = (i % 2 == 0) ? 360 : 0;
                if (hue == 0) {
                    ledBuffer.setHSV(i, 360, 0, 255);
                } else {
                    ledBuffer.setHSV(i, hue, 255, 255);
                }
            }
        });

    }

    public Command loading() {
        return runOnce(() -> {
            for (int i = 0; i < 2 && i > 8; i++) {
                final int hue = (i < 2 && i > 8) ? 180 : 0;
                if (hue == 0) {
                    ledBuffer.setHSV(i, 0, 0, 0);
                } else {
                    ledBuffer.setHSV(i, hue, 255, 255);
                }
            }
        });
    }

    public Command flash() {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final int hue = (i % 2 == 0) ? 30 : 0;
                if (hue == 0) {
                    ledBuffer.setHSV(i, 0, 0, 0);
                } else {
                    ledBuffer.setHSV(i, hue, 255, 255);
                }
            }
        });
    }

    private int allianceOffset = 0;

    public Command alliance(boolean isRedAlliance) {
        return runOnce(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final int hue = 135 + (int) (15
                        * Math.sin(
                                Math.PI * ((i + this.allianceOffset * ledBuffer.getLength()) / ledBuffer.getLength())));
                ledBuffer.setHSV(i, hue, 255, 255);
            }
            this.allianceOffset += 1;
            this.allianceOffset %= 10;
        });
    }

    public Command LPI(Pose2d currentRobotPosition) {
        return runOnce(() -> {
            // Loop through all of the startingPositions in the array
            // closest position is point B
            // current position in point A
            Pose2d closestPosition = new Pose2d();

            double closestDistance = getDistance(currentRobotPosition, closestPosition).getAsDouble();

            for (Pose2d startingPosition : LEDConstants.startingPositions) {

                double currentDistance = getDistance(currentRobotPosition, startingPosition).getAsDouble();

                if (currentDistance < closestDistance) {
                    closestPosition = startingPosition;
                    closestDistance = currentDistance;
                }
            }
        });

    }
    // Find the color to represent our distance
    // Set our leds to that color using interpolate()

    private DoubleSupplier getDistance(Pose2d currentRobotPosition, Pose2d closestPosition) {
        return () -> currentRobotPosition.relativeTo(closestPosition).getTranslation().getNorm();
    }

    // private double getDistance(Pose2d currentRobotPosition, Pose2d
    // closestPosition) {
    // // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method
    // 'getDistance'");
    // } **ask creator what this is**

    /**
     * Interpolate between two colors
     * 
     * @param color1 The first color
     * @param color2 The second color
     * @param t      The interpolation t (0, 1)
     */
    private static Color interpolate(Color color1, Color color2, double t) {
        if (t > 1d) {
            t = 1d;
        } else if (t < 0d) {
            t = 0d;
        }
        int red = (int) (color1.red * (1 - t) + color2.red * t);
        int green = (int) (color1.green * (1 - t) + color2.green * t);
        int blue = (int) (color1.blue * (1 - t) + color2.blue * t);
        return new Color(red, green, blue);
    }
}
