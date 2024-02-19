package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PieceControl;
import frc.robot.util.Neo;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Neo.TelemetryPreference;
import monologue.Logged;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase implements Logged {
    private final Neo intakeMotor;
    @Log
    private double desiredSpeed = 0;
    @Log
    private boolean notePossession = true; 

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

    public Command stop() {
        return setPercentCommand(IntakeConstants.STOP_PERCENT);
    }

    public boolean getPossession() {
        return this.notePossession;
    }

    public void setPossession(boolean possession) {
        this.notePossession = possession;
    }

    public Trigger possessionTrigger() {
        return new Trigger(this::getPossession);
    }

}
