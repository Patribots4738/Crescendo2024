package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
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
    public static final HashMap<Integer, Command> patternMap = new HashMap<>() {{
        put(0, "OFF");
        put(1, "GREENNGOLD");
        put(2, "CIRCUS");
        put(3, "LOADING");
        put(4, "LPI");
        put(5, "BLUE_ALLIANCE");
        put(6, "RED_ALLIANCE");
        put(7, "FLASH");
    }};

    private int currentPatternIndex;
    private final Supplier<Pose2d> poseSupplier;
    
    public LedStrip(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        this.led = new AddressableLED(LEDConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        
        led.setLength(ledBuffer.getLength());
        
        led.setData(ledBuffer);
        led.start();

        configureLED();

        greenNGold();
    }

    @Override
    public void periodic() {
        runPattern(currentPatternIndex);
    }

    private void runPattern(int index) {
        String pattern = patternMap.get(index);
        switch (pattern) {
            case "OFF"       -> turnOff();
            case "GREENGOLD" -> greenNGold();
            case "CiRCUS"    -> circus();
            case "LOADING"   -> loading();
            case "LPI"       -> LPI(poseSupplier.get());
            case "ALLIANCE"  -> alliance(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)));
            case "FLASH"     -> flash();
        }
    }

    public void setCurrentPattern(int index) {
        this.currentPatternIndex = index;
    }

    public void incrementPatternIndex() {
        currentPatternIndex ++;
        currentPatternIndex %= patternMap.size();
    }

    public int getCurrentPattern() {
        return currentPatternIndex;
    }

    private void configureLED() {}

    public void turnOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, 0, 0, 0);
        }
    }

    private int rainbowOffset;
    private void rainbow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int hue = (rainbowOffset + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 50, 100);
        }
        rainbowOffset += 3;
        rainbowOffset %= 180;
    }

    // https://www.desmos.com/calculator/s5ylo1v9lt
    private int greenNGoldOffset = 0;
    public void greenNGold() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            final int hue = 135 + (int) (15 * Math.sin(Math.PI * ((i + greenNGoldOffset*ledBuffer.getLength()) / ledBuffer.getLength())));
            ledBuffer.setHSV(i, hue, 255, 255);
        }
        greenNGoldOffset += 1;
        greenNGoldOffset %= 10;
    }

    public void circus() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            final int hue = (i % 2 == 0) ? 360 : 0;
            if (hue == 0) {
                ledBuffer.setHSV(i, 360, 0, 255);
            } else {
            ledBuffer.setHSV(i, hue, 255, 255);
            }
        }
    }

    public void loading() {
        for (int i = 0; i < 2 && i > 8; i++) {
            final int hue = (i < 2 && i > 8) ? 180 : 0;
            if (hue == 0) {
                ledBuffer.setHSV(i, 0, 0, 0);
            } else {
                ledBuffer.setHSV(i, hue, 255, 255);
            }
        }
    }

    public void flash() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            final int hue = (i % 2 == 0) ? 30 : 0;
            if (hue == 0) {
                ledBuffer.setHSV(i, 0, 0, 0);
            } else {
                ledBuffer.setHSV(i, hue, 255, 255);
            }
        }
    }

    private int allianceOffset = 0;
    public void alliance(boolean isRedAlliance) {        
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            final int hue = 135 + (int) (15 * Math.sin(Math.PI * ((i + allianceOffset*ledBuffer.getLength()) / ledBuffer.getLength())));
            ledBuffer.setHSV(i, hue, 255, 255);
        } 
        allianceOffset += 1;
        allianceOffset %= 10;
    }

    public void LPI (Pose2d currentRobotPosition) {
        // Loop through all of the startingPositions in the array
        Pose2d closestPosition = new Pose2d();
        
        double closestDistance = getDistance(currentRobotPosition, closestPosition);
        
        for (Pose2d startingPosition : LEDConstants.startingPositions) {
            
            double currentDistance = getDistance(currentRobotPosition, startingPosition);
            
            if (currentDistance < closestDistance) {
                closestPosition = startingPosition;
                closestDistance = currentDistance;
            }
        }
        // Find the color to represent our distance

        // Set our leds to that color using interpolate()
    }

    private double getDistance(Pose2d pointA, Pose2d pointB) { 
        return pointA.relativeTo(pointB).getTranslation().getNorm();
    }

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
