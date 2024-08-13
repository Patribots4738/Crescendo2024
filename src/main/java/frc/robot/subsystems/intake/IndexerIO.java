package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    class IndexerIOInputs {

        public double targetPercent = 0.0;
        public double positionRotations = 0.0;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;

    }

    default void updateInputs(IndexerIOInputs inputs) {}

}
