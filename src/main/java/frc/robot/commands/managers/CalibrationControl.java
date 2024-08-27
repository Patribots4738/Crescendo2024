package frc.robot.commands.managers;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.custom.SpeedAngleTriplet;

public class CalibrationControl {

    private SpeedAngleTriplet desiredTriplet = SpeedAngleTriplet.of(3311.0, 3034.0, 30.3);

    @AutoLogOutput (key = "Managers/CalibrationControl/LeftSpeed")
    private double leftSpeed = 3311;
    
    @AutoLogOutput (key = "Managers/CalibrationControl/RightSpeed")
    private double rightSpeed = 3034;

    @AutoLogOutput (key = "Managers/CalibrationControl/Angle")
    private double angle = 30;

    @AutoLogOutput (key = "Managers/CalibrationControl/PivotLock")
    private boolean pivotLock;
    
    @AutoLogOutput (key = "Managers/CalibrationControl/LeftLock")
    private boolean leftLock;
    
    @AutoLogOutput (key = "Managers/CalibrationControl/RightLock")
    private boolean rightLock;

    @AutoLogOutput (key = "Managers/CalibrationControl/Distance")
    private double distance = 0;

    private ShooterCmds shooterCmds;

    public CalibrationControl(ShooterCmds shooterCmds) {
        this.shooterCmds = shooterCmds;
    }   

    public Command copyCalcTriplet(Supplier<Pose2d> poseSupplier) {
        return Commands.runOnce(() -> {
            SpeedAngleTriplet triplet = shooterCmds.shooterCalc.calculateSpeakerTriplet(poseSupplier.get().getTranslation());
            desiredTriplet = triplet;
            leftSpeed = triplet.getLeftSpeed();
            rightSpeed = triplet.getRightSpeed();
            angle = triplet.getAngle();
            System.out.println("Copied triplet for " + Math.round(Units.metersToFeet(poseSupplier.get().getTranslation().getDistance(FieldConstants.GET_SPEAKER_TRANSLATION()))) + " ft");
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
