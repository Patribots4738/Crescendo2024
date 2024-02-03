package frc.robot.subsystems.leds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface LedStripIO extends Subsystem {

    Command turnOff();

    Command greenNGold();

    Command circus();

    Command loading();

    Command flash();

    void setLED(int startIndex, int endIndex, Color color);

    void setLED(int index, Color color);

    void setLED(Color color);

}
