package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CalibrationControl implements CalibrationControlIO {
    private HashMap<Integer, SpeedAngleTriplet> hashMap;
    private SpeedAngleTriplet currentVal;

    private boolean leftLocked;
    private boolean rightLocked;
    private boolean pivotLocked;

    public CalibrationControl() {
        hashMap = new HashMap<Integer, SpeedAngleTriplet>();
        currentVal = new SpeedAngleTriplet();
        leftLocked = false;
        rightLocked = false;
        pivotLocked = false;
    }

    @Override
    public HashMap<Integer, SpeedAngleTriplet> getHashMap() {
        return this.hashMap;
    }

    @Override
    public Command incrementLeftSpeed() {
        return Commands.runOnce(
            () -> currentVal = 
                SpeedAngleTriplet.of(
                    currentVal.getLeftSpeed() + 1, 
                    currentVal.getRightSpeed(), 
                    currentVal.getAngle()));
    }

    @Override
    public Command decrementLeftSpeed() {
        return Commands.runOnce(
            () -> currentVal = 
                SpeedAngleTriplet.of(
                    currentVal.getLeftSpeed() - 1, 
                    currentVal.getRightSpeed(), 
                    currentVal.getAngle()
                ));
    }

    @Override
    public Command incrementRightSpeed() {
        return Commands.runOnce(
            () -> currentVal = 
                SpeedAngleTriplet.of(
                    currentVal.getLeftSpeed(),
                    currentVal.getRightSpeed() + 1,
                    currentVal.getAngle()
                ));
    }

    @Override
    public Command decrementRightSpeed() {
        return Commands.runOnce(
            () -> currentVal =
                SpeedAngleTriplet.of(
                    currentVal.getLeftSpeed(), 
                    currentVal.getRightSpeed() - 1, 
                    currentVal.getAngle()
                ));
    }

    @Override
    public Command incrementBothSpeeds() {
        return Commands.sequence(
            incrementLeftSpeed(),
            incrementRightSpeed()
        );
    }

    @Override
    public Command decrementBothSpeeds() {
        return Commands.sequence(
            decrementLeftSpeed(),
            decrementRightSpeed()
        );
    }

    @Override
    public Command logAll() {
        return Commands.runOnce(
            () -> System.out.println(currentVal)
        );
    }

    @Override
    public Command incrementAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'incrementAngle'");
    }

    @Override
    public Command decrementAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'decrementAngle'");
    }

    @Override
    public Command lockBothSpeeds() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'lockBothSpeeds'");
    }

    @Override
    public Command lockLeftSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'lockLeftSpeed'");
    }

    @Override
    public Command lockRightSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'lockRightSpeed'");
    }

    @Override
    public Command lockPivotAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'lockPivotAngle'");
    }
}
