package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;
import frc.robot.util.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    /** Creates a new shooter. */
    private final Neo motorLeft;
    private final Neo motorRight;

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
        // This method will be called once per scheduler run
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
        motorLeft.setTargetVelocity(
                MathUtil.clamp(
                    speeds.getFirst(), 
                    ShooterConstants.SHOOTER_RPM_UPPER_LIMIT,
                    ShooterConstants.SHOOTER_RPM_LOWER_LIMIT));
        motorRight.setTargetVelocity(
                MathUtil.clamp(
                    speeds.getSecond(), 
                    ShooterConstants.SHOOTER_RPM_UPPER_LIMIT, 
                    ShooterConstants.SHOOTER_RPM_LOWER_LIMIT));
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
        return Commands.runOnce(() -> motorLeft.stopMotor());
    }
}
