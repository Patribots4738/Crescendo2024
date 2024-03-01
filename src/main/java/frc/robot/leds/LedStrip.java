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
                final Color color = (i < 2 && i > 8) ? Color.kFirstBlue : Color.kBlack;
                setLED(i, color);
            }
        });
    }

}