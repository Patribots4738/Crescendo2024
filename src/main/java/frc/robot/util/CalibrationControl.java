package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CalibrationControl {
    private SpeedAngleTriplet currentVal;

    private BooleanSupplier leftLocked;
    private BooleanSupplier rightLocked;
    private BooleanSupplier pivotLocked;

    private double distance = 0;

    public CalibrationControl() {
        currentVal = new SpeedAngleTriplet();
        leftLocked = () -> false;
        rightLocked = () -> false;
        pivotLocked = () -> false;
    }
    
    public Command changeLeftSpeed(double increment) {
        if(!leftLocked.getAsBoolean()) {
            return Commands.runOnce(
                () -> currentVal = SpeedAngleTriplet.of(
                    currentVal.getLeftSpeed() + increment,
                    currentVal.getRightSpeed(),
                    currentVal.getAngle()));
        } else {
            return Commands.none();
        }
    }

    public Command incrementLeftSpeed() {
        return changeLeftSpeed(100);
    }

    public Command decrementLeftSpeed() {
        return changeLeftSpeed(-100);
    }
    
    public Command changeRightSpeed(double increment) {
        if(!rightLocked.getAsBoolean()) {
            return Commands.runOnce(
                () -> currentVal = SpeedAngleTriplet.of(
                        currentVal.getLeftSpeed(),
                        currentVal.getRightSpeed() + increment,
                        currentVal.getAngle()));
        } else {
            return Commands.none();
        }
    }

    public Command incrementRightSpeed() {
        return changeRightSpeed(100);
    }

    public Command decrementRightSpeed() {
        return changeRightSpeed(-100);
    }

    public Command incrementBothSpeeds() {
        if(!leftLocked.getAsBoolean() && !rightLocked.getAsBoolean()) {
            return Commands.sequence(
                incrementLeftSpeed(),
                incrementRightSpeed());
        } else {
            return Commands.none();
        }
    }

    public Command decrementBothSpeeds() {
        if(!leftLocked.getAsBoolean() && !rightLocked.getAsBoolean()) {
            return Commands.sequence(
                decrementLeftSpeed(),
                decrementRightSpeed());
        } else {
            return Commands.none();
        }
    }
    
    public Command logDistance() {
        return Commands.runOnce(
            () -> { 
                System.out.println("Distance: " + distance); 
            });
    }

    public Command logAll() {
        return Commands.runOnce(
                () -> System.out.println("Distance: " + distance + ", Speeds: ("+currentVal.getLeftSpeed()+", "+currentVal.getRightSpeed()+"), Angle: ("+currentVal.getAngle()+")"));
    }

    public Command incrementAngle() {
        if(!pivotLocked.getAsBoolean()) {
            return Commands.runOnce(
                () -> currentVal = SpeedAngleTriplet.of(
                        currentVal.getLeftSpeed(),
                        currentVal.getRightSpeed(),
                        currentVal.getAngle() + 10));
        } else {
            return Commands.none();
        }
    }

    public Command decrementAngle() {
        if(!pivotLocked.getAsBoolean()) {
            return Commands.runOnce(
                () -> currentVal = SpeedAngleTriplet.of(
                        currentVal.getLeftSpeed(),
                        currentVal.getRightSpeed(),
                        currentVal.getAngle() - 10));
        } else {
            return Commands.none();
        }
    }

    public Command lockLeftSpeed() {
        return Commands.runOnce(
                () -> {
                    System.out.println("Locking Left");
                    leftLocked = () -> true;
                });
    }

    public Command unlockLeftSpeed() {
        return Commands.runOnce(() -> {
            System.out.println("Unlocking Left");
            leftLocked = () -> false;
        });
    }

    public Command lockRightSpeed() {
        return Commands.runOnce(() -> {
            System.out.println("Locking Right");
            rightLocked = () -> true;
        });
    }

    public Command unlockRightSpeed() {
        return Commands.runOnce(() -> {
            System.out.println("Unlocking Right");
            rightLocked = () -> false;
        });
    }

    public Command lockBothSpeeds() {
        return Commands.sequence(
                lockLeftSpeed(),
                lockRightSpeed());
    }

    public Command unlockBothSpeeds() {
        return Commands.sequence(
                unlockLeftSpeed(),
                unlockRightSpeed());
    }

    public Command lockPivotAngle() {
        return Commands.runOnce(() -> {
            System.out.println("Locking Pivot");
            pivotLocked = () -> true;
        });
    }

    public Command unlockPivotAngle() {
        return Commands.runOnce(() -> {
            System.out.println("Unlocking Pivot");
            pivotLocked = () -> false;
        });
    }

    public Command increaseDistance() {
        return Commands.sequence(
            Commands.runOnce(
                () -> { distance++; }),
            logDistance()
        );
    }

    public Command decreaseDistance() {
        return Commands.sequence(
            Commands.runOnce(
                () -> { distance--; }),
            logDistance()
        );
    }

}
