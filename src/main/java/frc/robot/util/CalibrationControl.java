package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShooterCalc;
import frc.robot.subsystems.shooter.Shooter;
import monologue.Logged;
import monologue.Annotations.Log;

public class CalibrationControl implements Logged {

    private SpeedAngleTriplet currentVal;

    @Log
    private boolean leftLocked;
    @Log
    private boolean rightLocked;
    @Log
    private boolean pivotLocked;

    @Log
    private double distance = 0;

    private ShooterCalc shooterCalc;

    public CalibrationControl(ShooterCalc shooterCalc) {
        currentVal = new SpeedAngleTriplet(3000.0, 3000.0, 60.0);
        leftLocked = false;
        rightLocked = false;
        pivotLocked = false;
        this.shooterCalc = shooterCalc;
    }
    
    public Command changeLeftSpeed(double increment) {
        if(!leftLocked) {
            return Commands.runOnce(
                () -> {currentVal = SpeedAngleTriplet.of(
                    currentVal.getLeftSpeed() + increment,
                    currentVal.getRightSpeed(),
                    currentVal.getAngle());
                    shooterCalc.setTriplet(currentVal); })
        .andThen(logAll());
        } else {
            return Commands.none();
        }
    }

    public void periodic() {
        shooterCalc.setTriplet(currentVal);
    }

    public Command incrementLeftSpeed() {
        return changeLeftSpeed(50);
    }

    public Command decrementLeftSpeed() {
        return changeLeftSpeed(-50);
    }
    
    public Command changeRightSpeed(double increment) {
        if(!rightLocked) {
            return Commands.runOnce(
                () -> {currentVal = SpeedAngleTriplet.of(
                        currentVal.getLeftSpeed(),
                        currentVal.getRightSpeed() + increment,
                        currentVal.getAngle());
                        shooterCalc.setTriplet(currentVal); })
            .andThen(logAll());
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
        if(!leftLocked && !rightLocked) {
            return Commands.sequence(
                incrementLeftSpeed(),
                incrementRightSpeed());
        } else {
            return Commands.none();
        }
    }

    public Command decrementBothSpeeds() {
        if(!leftLocked && !rightLocked) {
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
                System.out.println("Distance: " + distance + "ft"); 
            });
    }

    public Command logAll() {
        return Commands.runOnce(
                () -> System.out.println("put(" + distance + ", SpeedAngleTriplet.of("+currentVal.getLeftSpeed()+", "+currentVal.getRightSpeed()+", "+currentVal.getAngle()+"));"));
    }

    public Command incrementAngle() {
        return Commands.either(
            Commands.runOnce(
                () -> {currentVal = SpeedAngleTriplet.of(
                        currentVal.getLeftSpeed(),
                        currentVal.getRightSpeed(),
                        currentVal.getAngle() + 2.5);
                        shooterCalc.setTriplet(currentVal); })
            .andThen(logAll()),
            Commands.none(),
            () -> !pivotLocked);
    }

    public Command decrementAngle() {
        return Commands.either(
            Commands.runOnce(
                () -> {currentVal = SpeedAngleTriplet.of(
                        currentVal.getLeftSpeed(),
                        currentVal.getRightSpeed(),
                        currentVal.getAngle() - 2.5);
                        shooterCalc.setTriplet(currentVal); })
            .andThen(logAll()), 
            Commands.none(), 
            () -> !pivotLocked);
    }

    public Command toggleLeftSpeed() {
        return Commands.runOnce(() -> {
            leftLocked = !leftLocked;
            System.out.println("Pivot: "+pivotLocked+" | Left: "+leftLocked+" | Right: "+rightLocked+"");
        });
    }

    public Command toggleRightSpeed() {
        return Commands.runOnce(() -> {
            rightLocked = !rightLocked;
            System.out.println("Pivot: "+pivotLocked+" | Left: "+leftLocked+" | Right: "+rightLocked+"");
        });
    }

    public Command togglePivot() {
        return Commands.runOnce(() -> {
            pivotLocked = !pivotLocked;
            System.out.println("Pivot: "+pivotLocked+" | Left: "+leftLocked+" | Right: "+rightLocked+"");
        });
    }

    public Command toggleBothSpeeds() {
        return Commands.sequence(
                toggleLeftSpeed(),
                toggleRightSpeed());
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
