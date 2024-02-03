package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;

public interface PivotIO {
    void setAngle(double angle);

    Command setAngleCommand(double angle);

    void setRestAngle();

    Command setRestAngleCommand();

    double getAngle();

    Command stop();
}