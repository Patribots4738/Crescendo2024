package frc.robot.leds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.leds.commands.animations.LEDCommands;
import frc.robot.leds.commands.animations.LEDConditionalCommand;
import frc.robot.util.Constants.LEDConstants;

import java.util.List;

public class LedStrip extends SubsystemBase {

	private AddressableLED led;
    public AddressableLEDBuffer ledBuffer;

    private BooleanSupplier isRedAlliance = Robot::isRedAlliance;
    
    private LEDCommands ledFunctionCommand = new LEDCommands(this);

    public LedStrip() {
        this.led = new AddressableLED(LEDConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
    
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
        led.setData(ledBuffer);
    }

    public void setLED(int startIndex, int endIndex, Color color) {
        for (int i = startIndex; i < endIndex; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void setLED(int index, Color color) {
        if (index == -1) {
            setLED(color);
        } else {
            setLED(index, index+1, color);
        }
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

    public LEDCommands getLEDCommands() {
        return ledFunctionCommand;
    }
    
    //TODO: remove circus and loading when the commands have been made
    double circusOffset = 1;
    public Command circus() {
        return run(() -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final Color color = (((int) ((i/5.0) + circusOffset) % 2 == 0) ? Color.kRed : Color.kWhite);
                setLED(i, color);
            }
            circusOffset += .05;
        });
    }

    public Command loading() {
        return run(() -> {
            for (int i = 0; i < 2 && i > 8; i++) {
              int index = i;
              new LEDConditionalCommand(
                this.getLEDCommands(), 
                Color.kFirstBlue, 
                Color.kBlack, 
                () -> (index < 2 && index > 8)).schedule();
            }
        });
    }

    public class PatriColors extends Color {

        /**
         * Creates a new PatriColors object with the specified HSV values.
         * 
         * @param h the hue value (0-360)
         * @param s the saturation value (0-100)
         * @param v the value/brightness value (0-100)
         */
        public PatriColors(int h, int s, int v) {
            PatriColors.fromHSV360(h, s, v);
        }

        /**
         * Creates a Color from HSV values.
         *
         * @param h The h value [0-360)
         * @param s The s value [0-255]
         * @param v The v value [0-255]
         * @return The color
         */
        public static Color fromHSV360(int h, int s, int v) {
            // Loosely based on
            // https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
            // The hue range is split into 60 degree regions where in each region there
            // is one rgb component at a low value (m), one at a high value (v) and one
            // that changes (X) from low to high (X+m) or high to low (v-X)

            // Difference between highest and lowest value of any rgb component
            final int chroma = (s * v) / 255;

            // Because hue is 0-360 rather than 0-180 use 60 not 30
            final int region = (h / 60) % 6;

            // Remainder converted from 0-30 to 0-255
            final int remainder = (int) Math.round((h % 30) * (255 / 30.0));

            // Value of the lowest rgb component
            final int m = v - chroma;

            // Goes from 0 to chroma as hue increases
            final int X = (chroma * remainder) >> 8;

            switch (region) {
                case 0:
                    return new Color(v, X + m, m);
                case 1:
                    return new Color(v - X, v, m);
                case 2:
                    return new Color(m, v, X + m);
                case 3:
                    return new Color(m, v - X, v);
                case 4:
                    return new Color(X + m, m, v);
                default:
                    return new Color(v, m, v - X);
            }
        }
    }
}