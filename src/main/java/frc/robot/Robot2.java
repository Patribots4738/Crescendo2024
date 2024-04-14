package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.Log;

public class Robot2 extends TimedRobot implements Logged {
    // private final DigitalOutput dio10 = new DigitalOutput(13);
    private DigitalOutput[] alldios = new DigitalOutput[26];
    @Log
    private boolean[] stateArray = new boolean[26];
    @Override
    public void robotInit() {
        for (int i = 0; i <= 25; i++) {
            alldios[i] = new DigitalOutput(i);
        }
        Monologue.setupMonologue(this, "Robot/Draggables", false, true);

        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);
        DriverStation.silenceJoystickConnectionWarning(true);
        // dio10.set(true);
    }

    @Log
    private boolean relayState = false;

    @Override
    public void robotPeriodic() {
        relayState = ((int) Timer.getFPGATimestamp()) % 2 == 0;
        for (int i = 0; i <= 25; i++) {
            alldios[i].set(relayState);
            stateArray[i] = alldios[i].get();
        }
        Monologue.updateAll();
    }
}
