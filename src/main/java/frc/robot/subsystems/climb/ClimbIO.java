package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    class ClimbIOInputs {

        public double leftPositionMeters = 0.0;
        public double leftTargetPositionMeters = 0.0;
        public double leftAppliedVolts = 0.0;

        public double rightPositionMeters = 0.0;
        public double rightTargetPositionMeters = 0.0;
        public double rightAppliedVolts = 0.0;

    }

    default void updateInputs(ClimbIOInputs inputs) {}
    
}