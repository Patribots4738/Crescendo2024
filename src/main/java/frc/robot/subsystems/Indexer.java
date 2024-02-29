package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.rev.Neo;
import frc.robot.util.rev.SafeSparkMax.TelemetryPreference;

public class Indexer extends SubsystemBase {
    private final Neo indexer;
    private double desiredPercent = 0;

    public Indexer() {
        indexer = new Neo(IntakeConstants.TRIGGER_WHEEL_CAN_ID);
        indexer.setSmartCurrentLimit(IntakeConstants.TRIGGER_WHEEL_CURRENT_LIMIT_AMPS);
        indexer.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
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
            indexer.set(desiredPercent);
        }
    }

    public Command setPercentCommand(double percent) {
        return runOnce(() -> setDesiredPercent(percent));
    }

    public Command toShooter() {
        return setPercentCommand(IntakeConstants.SHOOTER_TRIGGER_WHEEL_PERCENT);
    }
    
    public Command toShooterSlow() {
        return setPercentCommand(IntakeConstants.SHOOTER_TRIGGER_WHEEL_PERCENT/2.0);
    }

    public Command toElevator() {
        return setPercentCommand(IntakeConstants.TRAP_TRIGGER_WHEEL_PERCENT);
    }

    public Command toElevatorSlow() {
        return setPercentCommand(.3);
    }

    public Command stopCommand() {
        return setPercentCommand(IntakeConstants.STOP_PERCENT);
    }

    public boolean hasPiece() {
        return false;
    }
}