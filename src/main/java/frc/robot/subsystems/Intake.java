package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.constants.Constants.IntakeConstants;
import frc.robot.util.motors.Neo;
import frc.robot.util.motors.SafeSparkMax.TelemetryPreference;
import monologue.Logged;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase implements Logged {
    private final Neo intakeMotor;

    @Log
    private double desiredSpeed = 0;
    private double startedIntakingTimestamp = 0;
    @Log
    private boolean notePossession = false; 

    public Intake() {
        intakeMotor = new Neo(IntakeConstants.INTAKE_CAN_ID);
        intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
        intakeMotor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    @Override
    public void periodic() {
        if (desiredSpeed == IntakeConstants.INTAKE_PERCENT
            && Robot.currentTimestamp - startedIntakingTimestamp > 0.6
            && intakeMotor.getVelocity() < 7000)
        {
            notePossession = true;
        }
    }

    public void setPercent(double desiredPercent) {
        desiredSpeed = 
            MathUtil.clamp(
                desiredPercent, 
                IntakeConstants.INTAKE_PERCENT_LOWER_LIMIT, 
                IntakeConstants.INTAKE_PERCENT_UPPER_LIMIT);
        intakeMotor.set(desiredSpeed);
    }

    public Command setPercentCommand(double desiredPercent) {
        return runOnce(() -> {
            setPercent(desiredPercent);
            if(desiredPercent == IntakeConstants.INTAKE_PERCENT) {
                startedIntakingTimestamp = Robot.currentTimestamp;
            }
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

    public boolean getPossession() {
        return notePossession;
    }

    public void setPossession(boolean possession) {
        this.notePossession = possession;
    }

    public Command revokePossesion() {
        return runOnce(() -> setPossession(false));
    }
}
