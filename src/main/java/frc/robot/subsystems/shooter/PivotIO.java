package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    class PivotIOInputs {
        public double desiredAngle = 0.0;
        public double realAngle = 0.0;
    }

    default void updateInputs(PivotIOInputs inputs) {}
    
}
