package frc.robot.subsystems.ampper;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    class ElevatorIOInputs {

        public double positionRotations = 0.0;
        public double targetPositionMeters = 0.0;
        public double appliedVolts = 0.0;

    }

    default void updateInputs(ElevatorIOInputs inputs) {}

}