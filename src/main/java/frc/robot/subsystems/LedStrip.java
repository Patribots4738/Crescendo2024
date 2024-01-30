package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.DriveConstants;
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
            patternMap.put(4, Commands.runOnce(() -> LPI(poseSupplier.get().getTranslation())));
            patternMap.put(5, Commands.runOnce(() -> alliance(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))));
            patternMap.put(6, Commands.runOnce(() -> flash()));
    }

    private Command runPattern(int index) {
        Command selectedPattern = switch (currentPatternIndex) {
            case (0) -> turnOff();
            case (1) -> greenNGold();
            case (2) -> circus();
            case (3) -> loading();
            case (4) -> LPI(poseSupplier.get().getTranslation());
            case (5) -> alliance(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)));
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

    public IntSupplier getCurrentPattern() {
        return () -> currentPatternIndex;
    }

    private Command configureLED() {
        return runOnce(() -> {
        });
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

    public Command LPI (Translation2d currentRobotPosition) {
        return runOnce(() -> {
            // Loop through all of the startingPositions in the array
            //closest position is point B
            //current position in point A
            Translation2d closestPosition = new Translation2d();

            double closestDistance = currentRobotPosition.getDistance(closestPosition);
            
            for (Pose2d startingPosition : LEDConstants.startingPositions) {
                
                double currentDistance = currentRobotPosition.getDistance(startingPosition.getTranslation());
                
                if (currentDistance < closestDistance) {
                    closestPosition = startingPosition.getTranslation(); 
                    closestDistance = currentDistance; 
                }
            }

            setZone(closestDistance);
        });
    }
    
    // Find the color to represent our distance
    // Set our leds to that color using interpolate()
    
    //https://www.tldraw.com/r/borcubmSklQQYMLikl6YZ?viewport=-1638,-855,3094,1889&page=page:page
    private void setZone(double distance) {
        if (distance > LEDConstants.OUTER_ZONE) {
            turnOff();
        } else if (distance > LEDConstants.INNER_ZONE
                && distance < LEDConstants.OUTER_ZONE) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setHSV(i, 255, 0, 0); // red
                ledBuffer.setHSV(i, 255, 0, 0); // red
            }
        } else if (distance > LEDConstants.RIN_STAR_BIN
                && distance < LEDConstants.INNER_ZONE) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {

                ledBuffer.setHSV(i, 60, 100, 100); // yellow

                ledBuffer.setHSV(i, 60, 100, 100); // yellow
            }
        } else if (distance < LEDConstants.RIN_STAR_BIN) {

            for (Translation2d wheelTranslation : DriveConstants.WHEEL_POSITION_ARRAY) {
                
            }

            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setHSV(i, 120, 100, 100); // green
            }
        }
    }
}
