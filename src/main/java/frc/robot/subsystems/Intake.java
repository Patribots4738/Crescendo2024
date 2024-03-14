package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.rev.Neo;
import frc.robot.util.rev.SafeSpark.TelemetryPreference;
import monologue.Logged;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase implements Logged {
    private final Neo motor;

    @Log
    private double desiredSpeed = 0;
    @Log
    private boolean notePossession = FieldConstants.IS_SIMULATION;

    public Intake() {
        motor = new Neo(IntakeConstants.INTAKE_CAN_ID, false);
        motor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
        motor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
        motor.makePossessionTrigger(IntakeConstants.INTAKE_PERCENT, 8000, 21, 0.3);
    }

    public void setPercent(double desiredPercent) {
        desiredPercent = 
            MathUtil.clamp(
                desiredPercent, 
                IntakeConstants.INTAKE_PERCENT_LOWER_LIMIT, 
                IntakeConstants.INTAKE_PERCENT_UPPER_LIMIT);
        
        if (desiredSpeed != desiredPercent) {
            desiredSpeed = desiredPercent;
            
            motor.set(desiredSpeed);
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

    public Command outCommand() {
        return setPercentCommand(IntakeConstants.OUTTAKE_PERCENT);
    }

    public boolean isStopped() {
        return desiredSpeed == 0;
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

    public Trigger getPossessionTrigger() {
        return motor.getPossessionTrigger();
    }

    public boolean hasNote() {
        return motor.hasNote();
    }
}
