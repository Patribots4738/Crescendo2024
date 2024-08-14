package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    class IntakeIOInputs {

        public double targetPercent = 0.0;
        public double positionRotations = 0.0;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double outputCurrentAmps = 0.0;

    }

    default void updateInputs(IntakeIOInputs inputs) {}

}