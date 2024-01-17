package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Neo;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Constants.NeoMotorConstants;

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
        intake.setInverted(false);

        NeoMotorConstants.motors.add(intake);
        
        //sets brake mode
        intake.setBrakeMode();
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
        return new Trigger(() -> (intake.getOutputCurrent() <= 7.0));
    }
        
}

