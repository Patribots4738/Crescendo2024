package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShooterCalc;
import monologue.Logged;
import monologue.Annotations.Log;

public class CalibrationControl implements Logged {

    private SpeedAngleTriplet desiredTriplet;

    @Log
    private double leftSpeed = 3200, rightSpeed = 3800, angle = 26;

    @Log
    private boolean pivotLock, leftLock, rightLock;

    @Log
    private double distance = 0;

    private ShooterCalc shooterCalc;

    public CalibrationControl(ShooterCalc shooterCalc) {
        this.shooterCalc = shooterCalc;
    }

    public Command incrementDistance(int increment) {
        return Commands.runOnce(() -> {
            distance += increment; 
            logDistance();
        });
    }

    public Command incrementSpeeds(DoubleSupplier increment) {
        return incrementLeftSpeed(increment).alongWith(incrementRightSpeed(increment));
    }

    public Command incrementLeftSpeed(DoubleSupplier increment) {
        return Commands.either(
            Commands.run(() -> {
                leftSpeed += increment.getAsDouble();
                logSpeeds();
            }),
            Commands.none(),
            () -> !leftLock
        );
    }

    public Command incrementRightSpeed(DoubleSupplier increment) {
        return Commands.either(
            Commands.run(() -> {
                rightSpeed += increment.getAsDouble();
                logSpeeds();
            }),
            Commands.none(),
            () -> !rightLock
        );
    }

    public Command incrementAngle(DoubleSupplier increment) {
        return Commands.either(
            Commands.run(() -> {
                angle = angle + increment.getAsDouble()/20.0;
                logAngle();
            }),
            Commands.none(),
            () -> !pivotLock
        );
    }
    
    public Command togglePivotLock() {
        return Commands.runOnce(() -> {
            pivotLock = !pivotLock;
            logLocks();
        });
    }

    public Command toggleLeftLock() {
        return Commands.runOnce(() -> {
            leftLock = !leftLock;
            logLocks();
        });
    }

    public Command toggleRightLock() {
        return Commands.runOnce(() -> {
            rightLock = !rightLock;
            logLocks();
        });
    }

    public void updateMotors() {
        leftSpeed = MathUtil.clamp(leftSpeed, 0, 5500);
        rightSpeed = MathUtil.clamp(rightSpeed, 0, 5500);
        angle = MathUtil.clamp(angle, 0, 60);
        desiredTriplet = new SpeedAngleTriplet(leftSpeed, rightSpeed, (double) Math.round(angle*10)/10.0);
        shooterCalc.setTriplet(desiredTriplet);
    }
    
    public Command updateMotorsCommand() {
        return Commands.runOnce(this::updateMotors);
    }

    public Command logTriplet() {
        return Commands.runOnce(
                () -> System.out.println("put(" + (int) distance + ", SpeedAngleTriplet.of("+desiredTriplet.getLeftSpeed()+", "+desiredTriplet.getRightSpeed()+", "+desiredTriplet.getAngle()+"));"));
    }

    public void logDistance() {
        System.out.println("Distance: " + (int) distance + "ft");
    }

    public void logSpeeds() {
        System.out.println("Speeds: " + leftSpeed + " | " + rightSpeed);
    }

    public void logAngle() {
        System.out.println("Angle: " + angle + "°");
    }

    public void logLocks() {
        System.out.println("Left: " + (leftLock ? "Locked  " : "Unlocked") + 
                           " | Pivot: " + (pivotLock ? "Locked  " : "Unlocked") + 
                           " | Right: " + (rightLock ? "Locked  " : "Unlocked"));
    }

}
