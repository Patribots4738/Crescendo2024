package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Neo;
import frc.robot.util.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final Neo topIntake;
    private final Neo bottomIntake;

    public Intake() {
        topIntake = new Neo(IntakeConstants.TOP_INTAKE_CAN_ID);
        bottomIntake = new Neo(IntakeConstants.BOTTOM_INTAKE_CAN_ID);
        configMotors();
    }

    public void configMotors() {
        bottomIntake.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        bottomIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        bottomIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        bottomIntake.setInverted(false); // TODO: is this correct?

        topIntake.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        topIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        topIntake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        topIntake.setInverted(false); // TODO: is this correct?

        // sets brake mode
        topIntake.setBrakeMode();
        bottomIntake.setBrakeMode();

        // TODO: should we slave the motors together?
    }

    public Command inCommand() {
        return runOnce(() -> {
            topIntake.set(IntakeConstants.INTAKE_SPEED);
            bottomIntake.set(IntakeConstants.INTAKE_SPEED);
        });
    }

    public Command outCommand() {
        return runOnce(() -> {
            topIntake.set(IntakeConstants.OUTTAKE_SPEED);
            bottomIntake.set(IntakeConstants.OUTTAKE_SPEED);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            topIntake.set(IntakeConstants.STOP_SPEED);
            bottomIntake.set(IntakeConstants.STOP_SPEED);
        });
    }

    public Trigger hasGamePieceTrigger() {
        return new Trigger(() -> topIntake.getOutputCurrent() > IntakeConstants.HAS_PIECE_CURRENT_THRESHOLD
                || bottomIntake.getOutputCurrent() > IntakeConstants.HAS_PIECE_CURRENT_THRESHOLD);
    }

}
