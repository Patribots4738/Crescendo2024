package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.rev.Neo;
import frc.robot.util.rev.SafeSpark.TelemetryPreference;
import monologue.Logged;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase implements Logged {
    private final Neo intakeMotor;

    @Log
    private double desiredSpeed = 0;
    
    public Intake() {
        intakeMotor = new Neo(IntakeConstants.INTAKE_CAN_ID, false);
        intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
        intakeMotor.setTelemetryPreference(TelemetryPreference.NO_ENCODER);
    }

    @Override
    public void periodic() {
        intakeMotor.updateHasControl(IntakeConstants.INTAKE_PERCENT, 0.3, 8000, 21);
    }

    public void setPercent(double desiredPercent) {
        desiredPercent = 
            MathUtil.clamp(
                desiredPercent, 
                IntakeConstants.INTAKE_PERCENT_LOWER_LIMIT, 
                IntakeConstants.INTAKE_PERCENT_UPPER_LIMIT);
        
        if (desiredSpeed != desiredPercent) {
            desiredSpeed = desiredPercent;
            if (desiredSpeed == IntakeConstants.INTAKE_PERCENT) {
                intakeMotor.setHasControl(false);
                intakeMotor.setPossiblyControlled();
            }
            
            intakeMotor.set(desiredSpeed);
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

    public boolean getPossession() {
        return intakeMotor.getHasControl();
    }

    public Command revokePossession() {
        return runOnce(() -> intakeMotor.setHasControl(false));
    }
}
