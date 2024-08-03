package frc.robot.subsystems.ampper;

import org.littletonrobotics.junction.AutoLog;

public interface AmpperIO {

    @AutoLog
    class AmpperIOInputs {

        public double positionRotations = 0.0;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;

    }

    default void updateInputs(AmpperIOInputs inputs) {}

}