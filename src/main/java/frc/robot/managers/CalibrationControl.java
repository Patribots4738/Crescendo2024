package frc.robot.managers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.SpeedAngleTriplet;
import monologue.Logged;
import monologue.Annotations.Log;

/**
 * This class is used to control the calibration of the shooter
 * for fast and easy tuning of the shooter speeds and angle.
 * 
 * The shooter is controlled by the SpeedAngleTriplet class
 * which contains the left and right speeds and the angle of the shooter.
 * 
 * The values are updated instantly and the shooter is updated.
 * After calibration, the values can be logged to the console for use in the code.
 */
public class CalibrationControl implements Logged {

    private SpeedAngleTriplet desiredTriplet = SpeedAngleTriplet.of(3311.0, 3034.0, 30.3);

    @Log
    private double leftSpeed = 3311, rightSpeed = 3034, angle = 30;

    @Log
    private boolean pivotLock, leftLock, rightLock;

    @Log
    private double distance = 0;

    private ShooterCmds shooterCmds;

    public CalibrationControl(ShooterCmds shooterCmds) {
        this.shooterCmds = shooterCmds;
    }

    /**
     * This command is used to copy the current SpeedAngleTriplet
     * to the desiredTriplet for use in the code.
     * 
     * @return Command that copies the current SpeedAngleTriplet and logs it
     */
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

    /**
     * The command that increments the distance of the shooter.
     * 
     * @param increment The amount to increment the distance by
     * @return          Command that increments the distance and logs it
     */
    public Command incrementDistance(int increment) {
        return Commands.runOnce(() -> {
            distance += increment;
            logDistance();
        });
    }

    /**
     * The command that increments the speeds of the shooter.
     * 
     * @param increment The amount to increment the speeds by
     * @return          Command that increments the speeds and logs them
     */
    public Command incrementSpeeds(DoubleSupplier increment) {
        return incrementLeftSpeed(increment).alongWith(incrementRightSpeed(increment));
    }

    /**
     * The command that increments the angle of the shooter.
     * 
     * @param increment The amount to increment the angle by
     * @return          Command that increments the angle and logs it
     */
    public Command incrementLeftSpeed(DoubleSupplier increment) {
        return Commands.either(
                Commands.run(() -> {
                    leftSpeed += increment.getAsDouble();
                    logSpeeds();
                }),
                Commands.none(),
                () -> !leftLock);
    }

    /**
     * The command that increments the angle of the shooter.
     * 
     * @param increment The amount to increment the angle by
     * @return          Command that increments the angle and logs it
     */
    public Command incrementRightSpeed(DoubleSupplier increment) {
        return Commands.either(
                Commands.run(() -> {
                    rightSpeed += increment.getAsDouble();
                    logSpeeds();
                }),
                Commands.none(),
                () -> !rightLock);
    }

    /**
     * The command that increments the angle of the shooter.
     * 
     * @param increment The amount to increment the angle by
     * @return          Command that increments the angle and logs it
     */
    public Command incrementAngle(DoubleSupplier increment) {
        return Commands.either(
                Commands.run(() -> {
                    angle = angle + increment.getAsDouble() / 20.0;
                    logAngle();
                }),
                Commands.none(),
                () -> !pivotLock);
    }
    
    /**
     * The Command to toggle the pivot lock.
     * 
     * @return Command that toggles the pivot lock and logs it
     */
    public Command togglePivotLock() {
        return Commands.runOnce(() -> {
            pivotLock = !pivotLock;
            logLocks();
        });
    }

    /**
     * The Command to toggle the left lock.
     * 
     * @return Command that toggles the left lock and logs it
     */
    public Command toggleLeftLock() {
        return Commands.runOnce(() -> {
            leftLock = !leftLock;
            logLocks();
        });
    }

    /**
     * The Command to toggle the right lock.
     * 
     * @return Command that toggles the right lock and logs it
     */
    public Command toggleRightLock() {
        return Commands.runOnce(() -> {
            rightLock = !rightLock;
            logLocks();
        });
    }

    /**
     * The void that updates the motors with the current SpeedAngleTriplet.
     * This ensures that the shooter is always up to date with the current values and is run every right button press.
     */
    public void updateMotors() {
        leftSpeed = MathUtil.clamp(leftSpeed, 0, 5500);
        rightSpeed = MathUtil.clamp(rightSpeed, 0, 5500);
        angle = MathUtil.clamp(angle, 0, 60);
        desiredTriplet = new SpeedAngleTriplet(leftSpeed, rightSpeed, (double) Math.round(angle*10)/10.0);
        shooterCmds.setTriplet(desiredTriplet);
    }
    
    /**
     * The command that updates the motors with the current SpeedAngleTriplet.
     * This ensures that the shooter is always up to date with the current values
     * and is run every right button press.
     */
    public Command updateMotorsCommand() {
        return Commands.runOnce(this::updateMotors);
    }

    /**
     * The command that logs the current SpeedAngleTriplet to the console.
     * This is used to get the values for the code after calibration.
     * 
     * @return Command that logs the current SpeedAngleTriplet
     */
    public Command logTriplet() {
        return Commands.runOnce(
                () -> System.out.println("put(" + (int) distance + ", SpeedAngleTriplet.of("
                        + Math.round(desiredTriplet.getLeftSpeed()) + ".0, "
                        + Math.round(desiredTriplet.getRightSpeed()) + ".0, " + desiredTriplet.getAngle() + "));"));
    }

    /**
     * The void that logs the distance to the console.
     */
    public void logDistance() {
        System.out.println("Distance: " + (int) distance + "ft");
    }

    /**
     * The void that logs the speeds to the console.
     */
    public void logSpeeds() {
        System.out.println("Speeds: " + leftSpeed + " | " + rightSpeed);
    }

    /**
     * The void that logs the angle to the console.
     */
    public void logAngle() {
        System.out.println("Angle: " + angle + "°");
    }

    /**
     * The void that logs the locks to the console.
     */
    public void logLocks() {
        System.out.println("Left: " + (leftLock ? "Locked  " : "Unlocked") + 
                           " | Pivot: " + (pivotLock ? "Locked  " : "Unlocked") + 
                           " | Right: " + (rightLock ? "Locked  " : "Unlocked"));
    }

}
