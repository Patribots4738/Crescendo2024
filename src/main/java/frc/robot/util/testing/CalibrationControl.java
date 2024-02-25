package frc.robot.util.testing;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.subsytemHelpers.ShooterCmds;
import frc.robot.util.constants.SpeedAngleTriplet;
import monologue.Logged;
import monologue.Annotations.Log;

public class CalibrationControl implements Logged {

    private SpeedAngleTriplet desiredTriplet = new SpeedAngleTriplet(0.0,0.0,0.0);

    @Log
    private double leftSpeed = 3200, rightSpeed = 3800, angle = 26;

    @Log
    private boolean pivotLock, leftLock, rightLock;

    @Log
    private double distance = 0;

    private ShooterCmds shooterCmds;

    public CalibrationControl(ShooterCmds shooterCmds) {
        this.shooterCmds = shooterCmds;
    }

    public Command copyCalcTriplet() {
        return Commands.runOnce(() -> {
            SpeedAngleTriplet triplet = shooterCmds.getTriplet();
            desiredTriplet = triplet;
            leftSpeed = triplet.getLeftSpeed();
            rightSpeed = triplet.getRightSpeed();
            angle = triplet.getAngle();
            logSpeeds();
            logAngle();
        });
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
        shooterCmds.setTriplet(desiredTriplet);
    }
    
    public Command updateMotorsCommand() {
        return Commands.runOnce(this::updateMotors);
    }

    public Command logTriplet() {
        return Commands.runOnce(
                () -> System.out.println("put(" + (int) distance + ", SpeedAngleTriplet.of("+Math.round(desiredTriplet.getLeftSpeed())+".0, "+Math.round(desiredTriplet.getRightSpeed())+".0, "+desiredTriplet.getAngle()+"));"));
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
