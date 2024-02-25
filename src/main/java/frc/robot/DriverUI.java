package frc.robot;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import monologue.Logged;
import monologue.Annotations.Log;

public class DriverUI implements Logged {
    
    public DriverUI() {}
    
    @Log
    public static SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();
    
    @Log
    public static boolean freshCode = true;

    @Log
    public static Field2d field = new Field2d();

    public static double currentTimestamp = 0;
    public static double previousTimestamp = 0;

}
