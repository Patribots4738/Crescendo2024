package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;
import frc.robot.util.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase implements Logged {
    /** Creates a new shooter. */
    private final Neo motorLeft;
    private final Neo motorRight;

    @Log
    public double leftRPM = 0, rightRPM = 0, leftDesiredRPM = 0, rightDesiredRPM = 0;

    @Log
    public boolean atDesiredRPM = false;

    public Shooter() {

        motorLeft = new Neo(ShooterConstants.LEFT_SHOOTER_CAN_ID);
        motorRight = new Neo(ShooterConstants.RIGHT_SHOOTER_CAN_ID);

        configMotors();

    }

    public void configMotors() {
        motorLeft.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        motorRight.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        motorLeft.setPID(
                ShooterConstants.SHOOTER_P,
                ShooterConstants.SHOOTER_I,
                ShooterConstants.SHOOTER_D,
                ShooterConstants.SHOOTER_MIN_OUTPUT,
                ShooterConstants.SHOOTER_MAX_OUTPUT);

        motorRight.setPID(
                ShooterConstants.SHOOTER_P,
                ShooterConstants.SHOOTER_I,
                ShooterConstants.SHOOTER_D,
                ShooterConstants.SHOOTER_MIN_OUTPUT,
                ShooterConstants.SHOOTER_MAX_OUTPUT);
    }

    @Override
    public void periodic() {
        Pair<Double, Double> realRPM = getRPM();
        Pair<Double, Double> desiredRPM = getTargetRPM();
        leftRPM = realRPM.getFirst();
        rightRPM = realRPM.getSecond();
        leftDesiredRPM = desiredRPM.getFirst();
        rightDesiredRPM = desiredRPM.getSecond();
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
    }

    public void setSpeed(Pair<Double, Double> speeds) {
        double speedLeft = 
            MathUtil.clamp(
                speeds.getFirst(), 
                ShooterConstants.SHOOTER_RPM_LOWER_LIMIT,
                ShooterConstants.SHOOTER_RPM_UPPER_LIMIT);
        double speedRight = 
            MathUtil.clamp(
                speeds.getSecond(), 
                ShooterConstants.SHOOTER_RPM_LOWER_LIMIT,
                ShooterConstants.SHOOTER_RPM_UPPER_LIMIT);
        motorLeft.setTargetVelocity(speedLeft);
        motorRight.setTargetVelocity(speedRight);
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

    public Pair<Double, Double> getRPM() {
        return new Pair<Double, Double>(motorLeft.getVelocity(), motorRight.getVelocity());
    }

    public Pair<Double, Double> getTargetRPM() {
        return new Pair<Double, Double>(motorLeft.getTargetVelocity(), motorRight.getTargetVelocity());
    }

    /**
     * The function is a command that stops both motors
     * 
     * @return The method is returning a Command object.
     */
    public Command stop() {
        return Commands.runOnce(() -> motorLeft.stopMotor());
    }

    // TODO: Implement a way to get the RPM of the shooter
    /**
     * The function is a BooleanSupplier that represents the the condition of
     * the velocity of the motor being equal to its targetVelocity
     * 
     * 
     * @return The method is returning a BooleanSupplier that returns true if
     *         the current velocity of the motors is at the target velocity with a
     *         small tolerance
     */
    public BooleanSupplier atDesiredRPM() {
        return () -> (MathUtil.applyDeadband(
                Math.abs(
                        getRPM().getFirst() - getTargetRPM().getFirst()),
                ShooterConstants.SHOOTER_DEADBAND) == 0
                && MathUtil.applyDeadband(
                Math.abs(
                        getRPM().getSecond() - getTargetRPM().getSecond()),
                ShooterConstants.SHOOTER_DEADBAND) == 0);
    }
}
