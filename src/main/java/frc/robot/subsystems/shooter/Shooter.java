package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCalc;
import frc.robot.util.Neo;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.PIDNotConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase implements Logged{
    /** Creates a new shooter. */
    private final Neo motorLeft;
    private final Neo motorRight;

    public PIDNotConstants shooterPID;
    @Log
    private double targetLeftSpeed = 0;
    @Log
    private double targetRightSpeed = 0;

    @Log
    private double currentLeftSpeed = 0;
    @Log
    private double currentRightSpeed = 0;

    public Shooter() {

        motorLeft = new Neo(ShooterConstants.LEFT_SHOOTER_CAN_ID);
        motorRight = new Neo(ShooterConstants.RIGHT_SHOOTER_CAN_ID, true);

        configMotors();
        shooterPID = new PIDNotConstants(ShooterConstants.SHOOTER_PID, ShooterConstants.SHOOTER_FF, motorLeft.getPIDController());
    }

    public void configMotors() {
        motorLeft.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        motorRight.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        motorLeft.setPID(
                ShooterConstants.SHOOTER_PID,
                ShooterConstants.SHOOTER_MIN_OUTPUT,
                ShooterConstants.SHOOTER_MAX_OUTPUT);

        motorLeft.setFF(ShooterConstants.SHOOTER_FF);

        motorRight.setPID(
                ShooterConstants.SHOOTER_PID,
                ShooterConstants.SHOOTER_MIN_OUTPUT,
                ShooterConstants.SHOOTER_MAX_OUTPUT);

        motorRight.setFF(ShooterConstants.SHOOTER_FF);
    }

    @Override
    public void periodic() {
        currentLeftSpeed = motorLeft.getVelocity();
        currentRightSpeed = motorRight.getVelocity();
    }

    /**
     * The function sets both of the motors to a targetVelocity that is
     * the speed provided
     * 
     * @param speed A double representing the speed the motors should be set to
     */
    public void setSpeed(double speed) {
        motorLeft.setTargetVelocity(speed);
        motorRight.setTargetVelocity(speed);
        targetLeftSpeed = speed;
        targetRightSpeed = speed;
    }

    public void setSpeed(Pair<Double, Double> speeds) {
        motorLeft.setTargetVelocity(speeds.getFirst());
        motorRight.setTargetVelocity(speeds.getSecond());
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
        return new Pair<Double, Double>(motorLeft.getVelocity(), motorRight.getVelocity());
    }

    /**
     * The function is a command that stops both motors
     * 
     * @return The method is returning a Command object.
     */
    public Command stop() {
        return runOnce(() -> motorLeft.set(0))
            .andThen(runOnce(() -> motorRight.set(0)));
    }

    public PIDNotConstants getPIDNotConstants() {
        return this.shooterPID;
    }
}
