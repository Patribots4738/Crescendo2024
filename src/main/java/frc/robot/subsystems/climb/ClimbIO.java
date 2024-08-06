package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    class ClimbIOInputs {

        public double leftPositionRotations = 0.0;
        public double leftTargetPositionRotations = 0.0;
        public double leftAppliedVolts = 0.0;

        public double rightPositionRotations = 0.0;
        public double rightTargetPositionRotations = 0.0;
        public double rightAppliedVolts = 0.0;

    }

    default void updateInputs(ClimbIOInputs inputs) {}
    
}