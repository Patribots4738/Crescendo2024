package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.SafeSpark.TelemetryPreference;
import frc.robot.util.Neo;

public class Indexer extends SubsystemBase {
    private final Neo triggerWheel;
    private double desiredPercent = 0;

    public Indexer() {
        triggerWheel = new Neo(IntakeConstants.TRIGGER_WHEEL_CAN_ID);
        configMotor();
    }

    public void configMotor() {
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        triggerWheel.setSmartCurrentLimit(IntakeConstants.TRIGGER_WHEEL_CURRENT_LIMIT_AMPS);
        triggerWheel.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    public double getDesiredPercent() {
        return desiredPercent;
    }

    public void setDesiredPercent(double percent) {
        desiredPercent = MathUtil.clamp(
            percent, 
            IntakeConstants.INDEXER_PERCENT_LOWER_LIMIT, 
            IntakeConstants.INDEXER_PERCENT_UPPER_LIMIT);
        triggerWheel.set(desiredPercent);
    }

    public Command setPercentCommand(double percent) {
        return runOnce(() -> setDesiredPercent(percent));
    }

    public Command toShooter() {
        return setPercentCommand(IntakeConstants.SHOOTER_TRIGGER_WHEEL_PERCENT);
    }

    public Command toElevator() {
        return setPercentCommand(IntakeConstants.TRAP_TRIGGER_WHEEL_PERCENT);
    }

    public Command stopCommand() {
        return setPercentCommand(IntakeConstants.STOP_PERCENT);
    }
}