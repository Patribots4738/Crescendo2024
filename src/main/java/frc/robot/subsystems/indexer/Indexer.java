package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.rev.Neo;
import frc.robot.util.rev.SafeSpark.TelemetryPreference;

public class Indexer extends SubsystemBase {
    private final Neo motor;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer() {
        motor = new Neo(IntakeConstants.TRIGGER_WHEEL_CAN_ID, false);
        motor.setSmartCurrentLimit(IntakeConstants.TRIGGER_WHEEL_CURRENT_LIMIT_AMPS);
        motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        Logger.recordOutput("Indexer/VelocityRPM", inputs.velocityRPM);
        Logger.recordOutput("Indexer/TargetPercent", inputs.targetPercent);
    }

    public double getDesiredPercent() {
        return inputs.targetPercent;
    }

    public void setDesiredPercent(double percent) {
        percent = MathUtil.clamp(
            percent, 
            IntakeConstants.INDEXER_PERCENT_LOWER_LIMIT, 
            IntakeConstants.INDEXER_PERCENT_UPPER_LIMIT);

        if (percent != inputs.targetPercent) {
            inputs.targetPercent = percent;
            motor.set(inputs.targetPercent);
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

    public boolean hasPiece() {
        return false;
    }

    public void updateInputs(IndexerIOInputs inputs) {
        inputs.targetPercent = motor.getTargetPercent();
        inputs.velocityRPM = motor.getVelocity();
        inputs.positionRotations = motor.getPosition();
        inputs.appliedVolts = motor.getAppliedOutput();
    }
}