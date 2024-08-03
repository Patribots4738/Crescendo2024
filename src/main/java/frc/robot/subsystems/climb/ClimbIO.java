package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    class ClimbIOInputs {

        public double leftPosition = 0.0;
        public double leftTargetPosition = 0.0;
        public double leftAppliedVolts = 0.0;

        public double rightPosition = 0.0;
        public double rightTargetPosition = 0.0;
        public double rightAppliedVolts = 0.0;

    }

    default void updateInputs(ClimbIOInputs inputs) {}
    
}