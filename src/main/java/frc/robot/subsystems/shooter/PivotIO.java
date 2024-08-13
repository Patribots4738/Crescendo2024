package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    class PivotIOInputs {

        

    }

    default void updateInputs(PivotIOInputs inputs) {}
    
}
