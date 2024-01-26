package frc.robot;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import monologue.Logged;
import monologue.Annotations.Log;

public class DriverUI implements Logged {
    
    @Log.NT
    public static SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();
    
    @Log.NT
    public static boolean freshCode = true;

    @Log.NT
    public static Field2d field = new Field2d();

    public static double currentTimestamp = 0;
    public static double previousTimestamp = 0;

    public DriverUI() {
        addAutos();
    }
    
    private void addAutos() {
        // autoChooser.addOption()
    }

}
