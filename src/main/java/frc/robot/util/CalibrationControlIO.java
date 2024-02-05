package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;

public interface CalibrationControlIO {
    HashMap<Integer, SpeedAngleTriplet> hashMap = new HashMap<Integer, SpeedAngleTriplet>();
    SpeedAngleTriplet currentVal = new SpeedAngleTriplet();

    boolean leftLocked = false;
    boolean rightLocked = false;
    boolean pivotLocked = false;

    public HashMap<Integer, SpeedAngleTriplet> getHashMap();

    // by 100
    public Command incrementLeftSpeed();
    public Command decrementLeftSpeed();
    
    public Command incrementRightSpeed();
    public Command decrementRightSpeed();
    
    public Command incrementBothSpeeds();
    public Command decrementBothSpeeds();
    
    public Command logAll();

    public Command incrementAngle();
    public Command decrementAngle();

    public Command lockBothSpeeds();
    public Command lockLeftSpeed();
    public Command lockRightSpeed();

    public Command lockPivotAngle();
    
}
