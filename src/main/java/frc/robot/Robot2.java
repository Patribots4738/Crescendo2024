package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.Log;

public class Robot2 extends TimedRobot implements Logged {
    private final DigitalOutput dio10 = new DigitalOutput(10);
    @Log
    boolean state = false;
    @Override
    public void robotInit() {
        Monologue.setupMonologue(this, "Robot/Draggables", false, true);

        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);
        DriverStation.silenceJoystickConnectionWarning(true);
        dio10.set(true);
    }

    @Override
    public void robotPeriodic() {
        Monologue.updateAll();
        state = dio10.get();
    }
}
