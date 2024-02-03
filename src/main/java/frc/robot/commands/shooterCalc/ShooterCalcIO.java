package frc.robot.commands.shooterCalc;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface ShooterCalcIO {
    Command prepareFireCommand(BooleanSupplier shootAtSpeaker, Supplier<Pose2d> robotPose);

    Command prepareFireMovingCommand(BooleanSupplier shootAtSpeaker, Supplier<Pose2d> robotPose);

    Command sendBackCommand();

    Command resetShooter();

    Command stopMotors();
}