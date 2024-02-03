package frc.robot.subsystems.shooter.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;

public interface ShooterIO {
    void setSpeed(double speed);

    void setSpeed(Pair<Double, Double> speeds);

    Command setSpeedCommand(double speed);

    Command setSpeedCommand(Pair<Double, Double> speeds);

    Pair<Double, Double> getSpeed();

    Command stop();
}
