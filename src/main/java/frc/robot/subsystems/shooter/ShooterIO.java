package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {

        public double leftVelocityRPM = 0.0;
        public double leftTargetVelocityRPM = 0.0;
        public double leftPositionRotations = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftOutputCurrentAmps = 0.0;

        public double rightVelocityRPM = 0.0;
        public double rightTargetVelocityRPM = 0.0;
        public double rightPositionRotations = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightOutputCurrentAmps = 0.0;

    }

    default void updateInputs(ShooterIOInputs inputs) {}
    
}
