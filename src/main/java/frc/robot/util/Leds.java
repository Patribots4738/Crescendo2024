package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.Swerve;

public class Leds {
    
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private Swerve swerve;

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

    private int rainbowFirstPixelHue;
    private void rainbow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 50, 100);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    // https://www.desmos.com/calculator/s5ylo1v9lt
    private int greenNGoldOffset = 0;
    private void greenNGold() {
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
}
