package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Neo;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Neo.TelemetryPreference;

public class Intake extends SubsystemBase {
    private final Neo intakeMotor;
    private double desiredSpeed = 0;

    public Intake() {
        intakeMotor = new Neo(IntakeConstants.INTAKE_CAN_ID);
        configMotors();
    }

    public void configMotors() {
        intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        intakeMotor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    public void setPercent(double desiredPercent) {
        desiredSpeed = 
            MathUtil.clamp(
                desiredPercent, 
                IntakeConstants.INTAKE_PERCENT_LOWER_LIMIT, 
                IntakeConstants.INTAKE_PERCENT_UPPER_LIMIT);
        intakeMotor.setTargetPercent(desiredSpeed);
    }

    public Command setPercentCommand(double desiredPercent) {
        return runOnce(() -> setPercent(desiredPercent));
    }

    public Command inCommand() {
        return setPercentCommand(IntakeConstants.INTAKE_PERCENT);
    }

    public Command outCommand() {
        return setPercentCommand(IntakeConstants.OUTTAKE_PERCENT);
    }

    public boolean isStopped() {
        return desiredSpeed != 0;
    }

    public Command stopCommand() {
        return setPercentCommand(IntakeConstants.STOP_PERCENT);
    }

    public Trigger hasGamePieceTrigger() {
        return new Trigger(() -> intakeMotor.getOutputCurrent() > IntakeConstants.HAS_PIECE_CURRENT_THRESHOLD);
    }

}
