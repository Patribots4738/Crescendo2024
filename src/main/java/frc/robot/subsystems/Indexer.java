package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import lib.rev.Neo;
import lib.rev.SafeSpark.TelemetryPreference;

public class Indexer extends SubsystemBase {
    private final Neo motor;
    private double desiredPercent = 0;

    public Indexer() {
        motor = new Neo(IntakeConstants.TRIGGER_WHEEL_CAN_ID, false);
        motor.setSmartCurrentLimit(IntakeConstants.TRIGGER_WHEEL_CURRENT_LIMIT_AMPS);
        motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    public double getDesiredPercent() {
        return desiredPercent;
    }

    public void setDesiredPercent(double percent) {
        percent = MathUtil.clamp(
            percent, 
            IntakeConstants.INDEXER_PERCENT_LOWER_LIMIT, 
            IntakeConstants.INDEXER_PERCENT_UPPER_LIMIT);

        if (percent != desiredPercent) {
            desiredPercent = percent;
            motor.set(desiredPercent);
        }
    }

    public Command setPercentCommand(double percent) {
        return runOnce(() -> setDesiredPercent(percent));
    }

    public Command toShooter() {
        return setPercentCommand(IntakeConstants.SHOOTER_TRIGGER_WHEEL_PERCENT);
    }
    
    public Command toShooterSlow() {
        return setPercentCommand(IntakeConstants.SHOOTER_TRIGGER_WHEEL_PERCENT/1.5);
    }

    public Command toElevator() {
        return setPercentCommand(IntakeConstants.AMP_TRIGGER_WHEEL_PERCENT);
    }

    public Command toElevatorSlow() {
        return setPercentCommand(.3);
    }

    public Command stopCommand() {
        return setPercentCommand(IntakeConstants.STOP_PERCENT);
    }

    public Command setCoastMode() {
        return runOnce(() -> motor.setCoastMode());
    }

    public Command setBrakeMode() {
        return runOnce(() -> motor.setBrakeMode());
    }
}