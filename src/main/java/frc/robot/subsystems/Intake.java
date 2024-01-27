package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Neo;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final Neo intake;

    public Intake() {
        intake = new Neo(IntakeConstants.INTAKE_CAN_ID);
        configMotor();
    }

    public void configMotor() {
        intake.setSmartCurrentLimit(IntakeConstants.INTAKE_STALL_CURRENT_LIMIT_AMPS, IntakeConstants.INTAKE_FREE_CURRENT_LIMIT_AMPS);
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        intake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);

        //sets brake mode
        intake.setBrakeMode();
    }

    public void tick() {
        intake.tick();
    }

    @Override
    public void periodic() {
        if (FieldConstants.IS_SIMULATION) {
            tick();
        }
    }

    public Command inCommand() {
        return runOnce(() -> intake.set(IntakeConstants.INTAKE_SPEED));
    }

    public Command outCommand() {
        return runOnce(() -> intake.set(IntakeConstants.OUTTAKE_SPEED));
    }

    public Command stopCommand() {
        return runOnce(() -> intake.set(IntakeConstants.STOP_SPEED));
    }

    public Trigger hasGamePieceTrigger() {
        return new Trigger(() -> (intake.getOutputCurrent() > IntakeConstants.HAS_PIECE_CURRENT_THRESHOLD));
    }
        
}

