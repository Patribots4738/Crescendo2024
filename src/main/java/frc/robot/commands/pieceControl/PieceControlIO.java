package frc.robot.commands.pieceControl;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public interface PieceControlIO {
    Trigger readyToShoot();

    Command stopAllMotors();

    Command noteToShoot();

    Command noteToTarget(BooleanSupplier toAmp);

    Command placeTrapCommand();
}
