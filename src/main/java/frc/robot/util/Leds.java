package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.Swerve;

public class Leds {
    
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private Swerve swerve;

    private int rainbowFirstPixelHue;

    public Leds(Swerve swerve) {
        this.led = new AddressableLED(9);
        ledBuffer = new AddressableLEDBuffer(0);

        led.setLength(ledBuffer.getLength());
        
        led.setData(ledBuffer);
        led.start();

        configureLED();
        this.swerve = swerve;
    }

    private void configureLED() {

    }

    public void turnOn() {
        
    }

    public void turnOff() {

    }

    private void rainbow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 50, 100);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    private void greenNGold() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            for (int j = 0; j < ledBuffer.getLength(); j++) {
                final int hue = 0;
                ledBuffer.setHSV(j, hue, 50, 100);
            }
        }
    }
}
