package frc.robot.util.rev;

import org.littletonrobotics.junction.AutoLog;

public interface MAXSwerveModuleIO {

    @AutoLog
    class MAXSwerveModuleIOInputs {

    }

    default void updateInputs(MAXSwerveModuleIOInputs inputs) {}

}