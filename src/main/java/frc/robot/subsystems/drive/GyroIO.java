package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;


public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadsPerSec = 0.0;
    }

    default void updateInputs() {}
}
