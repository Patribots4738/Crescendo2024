package frc.robot.subsystems.ampper;

import org.littletonrobotics.junction.AutoLog;

public interface AmpperIO {

    @AutoLog
    class AmpperIOInputs {

        public double targetPercent = 0.0;
        public double appliedVolts = 0.0;
        public double outputCurrentAmps = 0.0;

    }

    default void updateInputs(AmpperIOInputs inputs) {}

}