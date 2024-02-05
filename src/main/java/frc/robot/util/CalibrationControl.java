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

    private DoubleSupplier distance = () -> 0;

    public CalibrationControl() {
        this.currentVal = new SpeedAngleTriplet();
        this.leftLocked = () -> false;
        this.rightLocked = () -> false;
        this.pivotLocked = () -> false;
    }
    
    public Command changeLeftSpeed(double increment) {
        if(!leftLocked.getAsBoolean()) {
            return Commands.runOnce(
                () -> this.currentVal = SpeedAngleTriplet.of(
                    this.currentVal.getLeftSpeed() + increment,
                    this.currentVal.getRightSpeed(),
                    this.currentVal.getAngle()));
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
                () -> this.currentVal = SpeedAngleTriplet.of(
                        this.currentVal.getLeftSpeed(),
                        this.currentVal.getRightSpeed() + increment,
                        this.currentVal.getAngle()));
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
    
    public Command logAll() {
        return Commands.runOnce(
                () -> System.out.println("Distance: " + this.distance.getAsDouble() + ", Values: " + this.currentVal));
    }

    public Command incrementAngle() {
        if(!pivotLocked.getAsBoolean()) {
            return Commands.runOnce(
                () -> this.currentVal = SpeedAngleTriplet.of(
                        this.currentVal.getLeftSpeed(),
                        this.currentVal.getRightSpeed(),
                        this.currentVal.getAngle() + 10));
        } else {
            return Commands.none();
        }
    }

    public Command decrementAngle() {
        if(!pivotLocked.getAsBoolean()) {
            return Commands.runOnce(
                () -> this.currentVal = SpeedAngleTriplet.of(
                        this.currentVal.getLeftSpeed(),
                        this.currentVal.getRightSpeed(),
                        this.currentVal.getAngle() - 10));
        } else {
            return Commands.none();
        }
    }

    public Command lockLeftSpeed() {
        return Commands.runOnce(
                () -> {
                    System.out.println("Locking Left");
                    this.leftLocked = () -> true;
                });
    }

    public Command unlockLeftSpeed() {
        return Commands.runOnce(() -> {
            System.out.println("Unlocking Left");
            this.leftLocked = () -> false;
        });
    }

    public Command lockRightSpeed() {
        return Commands.runOnce(() -> {
            System.out.println("Locking Right");
            this.rightLocked = () -> true;
        });
    }

    public Command unlockRightSpeed() {
        return Commands.runOnce(() -> {
            System.out.println("Unlocking Right");
            this.rightLocked = () -> false;
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
            this.pivotLocked = () -> true;
        });
    }

    public Command unlockPivotAngle() {
        return Commands.runOnce(() -> {
            System.out.println("Unlocking Pivot");
            this.pivotLocked = () -> false;
        });
    }

    public Command increaseDistance() {
        return Commands.runOnce(
                () -> {
                    System.out.println("Distance: " + this.distance.getAsDouble());
                    this.distance = () -> this.distance.getAsDouble() + 1;
                });
    }

    public Command decreaseDistance() {
        return Commands.runOnce(
                () -> {
                    System.out.println("Distance: " + this.distance.getAsDouble());
                    this.distance = () -> this.distance.getAsDouble() - 1;
                });
    }
}
