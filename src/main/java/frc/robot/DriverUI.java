package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import monologue.Logged;
import monologue.Annotations.Log;

public class DriverUI implements Logged {
    @Log.NT
    public static boolean freshCode = true;

    @Log.NT
    public static Field2d field = new Field2d();

    public static double currentTimestamp = 0;
    public static double previousTimestamp = 0;

    public DriverUI() {}
}
