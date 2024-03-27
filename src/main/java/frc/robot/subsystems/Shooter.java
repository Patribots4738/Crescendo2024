package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.rev.Neo;
import monologue.Logged;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase implements Logged{
    /** Creates a new shooter. */
    private final Neo leftMotor;
    private final Neo rightMotor;

    @Log
    private double targetLeftSpeed = 0;
    @Log
    private double targetRightSpeed = 0;

    @Log
    private double currentLeftSpeed = 0;
    @Log
    private double currentRightSpeed = 0;

    @Log
    private boolean atDesiredRPM = false;

    public Shooter() {

        leftMotor = new Neo(ShooterConstants.LEFT_SHOOTER_CAN_ID, true, true);
        rightMotor = new Neo(ShooterConstants.RIGHT_SHOOTER_CAN_ID, true);

        configMotors();
    }

    public void configMotors() {
        leftMotor.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        leftMotor.setPID(ShooterConstants.SHOOTER_PID);

        rightMotor.setPID(ShooterConstants.SHOOTER_PID);

        leftMotor.setCoastMode();
        rightMotor.setCoastMode();
    }

    @Override
    public void periodic() {
        currentLeftSpeed = leftMotor.getVelocity();
        currentRightSpeed = rightMotor.getVelocity();
        
        atDesiredRPM = 
            MathUtil.isNear(
                currentLeftSpeed, targetLeftSpeed,
                ShooterConstants.SHOOTER_RPM_DEADBAND)
            && MathUtil.isNear(
                currentRightSpeed, targetRightSpeed,
                ShooterConstants.SHOOTER_RPM_DEADBAND);
    }

    /**
     * The function sets both of the motors to a targetVelocity that is
     * the speed provided
     * 
     * @param speed A double representing the speed the motors should be set to
     */
    public void setSpeed(double speed) {
        leftMotor.setTargetVelocity(speed);
        rightMotor.setTargetVelocity(speed);
        targetLeftSpeed = speed;
        targetRightSpeed = speed;
    }

    public void setSpeed(Pair<Double, Double> speeds) {
        leftMotor.setTargetVelocity(speeds.getFirst());
        rightMotor.setTargetVelocity(speeds.getSecond());
        targetLeftSpeed = speeds.getFirst();
        targetRightSpeed = speeds.getSecond();
    }

    /**
     * The function is a command that sets the motor speed for both motors
     * to the speed provided
     * 
     * @param speed A double representing the speed the motors should be set to
     * 
     * @return The method is returning a Command object.
     */
    public Command setSpeedCommand(double speed) {
        return Commands.runOnce(() -> setSpeed(speed));
    }

    public Command setSpeedCommand(Pair<Double, Double> speeds) {
        return runOnce(() -> setSpeed(speeds));
    }

    public Pair<Double, Double> getSpeed() {
        return new Pair<Double, Double>(leftMotor.getVelocity(), rightMotor.getVelocity());
    }

    public double getAverageSpeed() {
        return (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2.0;
    }

    /**
     * The function is a command that stops both motors
     * 
     * @return The method is returning a Command object.
     */
    public Command stopCommand() {
        return Commands.either(
            setSpeedCommand(0),
            runOnce(() -> {
                leftMotor.set(0);
                rightMotor.set(0);
            }),
            () -> FieldConstants.IS_SIMULATION);
    }
    
    public boolean getAtDesiredRPM() {
        return atDesiredRPM;
    }

    public Pair<Double, Double> getDesiredSpeeds() {
        return new Pair<Double, Double>(targetLeftSpeed, targetRightSpeed);
    }

    public double getAverageTargetSpeed() {
        return (targetLeftSpeed + targetRightSpeed) / 2.0;
    }
}