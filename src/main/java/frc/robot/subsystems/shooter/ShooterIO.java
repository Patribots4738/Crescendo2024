package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {

        

    }

    default void updateInputs(ShooterIOInputs inputs) {}
    
}
