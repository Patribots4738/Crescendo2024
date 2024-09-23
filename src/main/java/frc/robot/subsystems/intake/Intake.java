package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Constants.LoggingConstants;
import frc.robot.util.rev.Neo;
import frc.robot.util.rev.SafeSpark.TelemetryPreference;

public class Intake extends SubsystemBase implements IntakeIO {
    private final Neo motor;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    @AutoLogOutput (key = "Subsystems/Intake/NotePossession")
    private boolean notePossession = FieldConstants.IS_SIMULATION;

    public Intake() {
        motor = new Neo(IntakeConstants.INTAKE_CAN_ID, false);
        motor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
        motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Intake", inputs);
    }

    public void setPercent(double desiredPercent) {
        desiredPercent = 
            MathUtil.clamp(
                desiredPercent, 
                IntakeConstants.INTAKE_PERCENT_LOWER_LIMIT, 
                IntakeConstants.INTAKE_PERCENT_UPPER_LIMIT);
        
        if (inputs.targetPercent != desiredPercent) {
            
            motor.set(desiredPercent);

        }

    }

    public Command setPercentCommand(double desiredPercent) {
        return runOnce(() -> {
            setPercent(desiredPercent);
        });
    }

    public Command inCommand() {
        return setPercentCommand(IntakeConstants.INTAKE_PERCENT);
    }

    public Command inCommandFor(double seconds) {
        return setPercentCommand(IntakeConstants.INTAKE_PERCENT)
            .andThen(
                Commands.waitSeconds(seconds),
                stopCommand());
    }

    public Command inCommandSlow() {
        return setPercentCommand(IntakeConstants.INTAKE_PERCENT/3.0);
    }

    public Command inCommandSlow(double speed) {
        return setPercentCommand(speed);
    }

    public Command outCommand() {
        return setPercentCommand(IntakeConstants.OUTTAKE_PERCENT);
    }

    public boolean isStopped() {
        return inputs.targetPercent == 0;
    }

    public Command stopCommand() {
        return setPercentCommand(IntakeConstants.STOP_PERCENT);
    }

    public Command setCoastMode() {
        return runOnce(() -> motor.setCoastMode()).ignoringDisable(true);
    }

    public Command setBrakeMode() {
        return runOnce(() -> motor.setBrakeMode()).ignoringDisable(true);
    }

    public double getDesiredSpeed() {
        return inputs.targetPercent;
    }

    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.targetPercent = motor.getTargetPercent();
        inputs.appliedVolts = motor.getAppliedOutput();
        inputs.outputCurrentAmps = motor.getOutputCurrent();
    }
}
