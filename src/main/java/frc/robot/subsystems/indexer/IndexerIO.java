package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;

public interface IndexerIO {
    double getDesiredSpeed();

    void setDesiredSpeed(double speed);

    Command setSpeedCommand(double speed);

    Command toShooter();

    Command toTrap();

    Command stop();
}