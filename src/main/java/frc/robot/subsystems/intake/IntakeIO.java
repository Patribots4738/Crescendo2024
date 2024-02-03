package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IntakeIO {
    Command setCommand(double desiredSpeed);

    Command inCommand();

    Command outCommand();

    Command stop();

    Trigger hasGamePieceTrigger();
}
