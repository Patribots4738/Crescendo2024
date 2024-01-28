package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.LEDConstants;

public class Leds {
    
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public Leds() {
        this.led = new AddressableLED(LEDConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        
        led.setLength(ledBuffer.getLength());
        
        led.setData(ledBuffer);
        led.start();

        configureLED();

        greenNGold();
    }

    private void configureLED() {}

    public void turnOn() {
        for (it i = 0; i < ledBuffer.getLength)
    }

    public void turnOff() {
        for (int i = 0; )
        ledBuffer.settHSV(i, 0, 0, 0);
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
        // We know that the hue for green is roughly 120
        // and the hue for gold is roughly 150
        // We need to make an equation that takes a number and oscilates it around another point.
        // Sounds like a sin wave to me. Let's do it!
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
}

