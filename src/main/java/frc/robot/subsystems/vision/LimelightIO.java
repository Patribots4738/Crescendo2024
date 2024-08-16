package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.calc.LimelightHelpers.Results;

public interface LimelightIO {

    @AutoLog
    class LimelightIOInputs {

        public Results results = new Results();

    }

    default void updateInputs(LimelightIOInputs inputs) {}
    
}
