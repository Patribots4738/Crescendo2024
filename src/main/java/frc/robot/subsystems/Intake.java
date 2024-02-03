package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LedStrip;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Neo;
import frc.robot.util.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final Neo intakeMotor;

    public Intake() {
        intakeMotor = new Neo(IntakeConstants.TOP_INTAKE_CAN_ID);
        configMotors();
    }

    public void configMotors() {
        intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    }

    LedStrip ledStrip = new LedStrip();

    public Command setCommand(double desiredSpeed) {
        return runOnce(() -> intakeMotor.set(desiredSpeed));
    }

    public Command inCommand() {
        return setCommand(IntakeConstants.INTAKE_SPEED)
            .alongWith(Commands.run(() -> ledStrip.setLED(Color.kBlue)))
                .andThen(Commands.run(() -> ledStrip.setLED(Color.kGreen)));
    }

    public Command outCommand() {
        return setCommand(IntakeConstants.OUTTAKE_SPEED)
            .alongWith(Commands.run(() -> ledStrip.setLED(Color.kBlanchedAlmond)))
                .andThen(Commands.run(() -> ledStrip.setLED(Color.kBrown)));
    }

    public Command stop() {
        return setCommand(IntakeConstants.STOP_SPEED)
            .alongWith(Commands.run(() -> ledStrip.setLED(Color.kRed)));
    }

    public Trigger hasGamePieceTrigger() {
        return new Trigger(() -> intakeMotor.getOutputCurrent() > IntakeConstants.HAS_PIECE_CURRENT_THRESHOLD);
    }

}
