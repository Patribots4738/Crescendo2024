package frc.robot.subsystems.colorsensor;

import org.littletonrobotics.junction.AutoLog;

public interface PicoColorSensorIO {

    @AutoLog
    class PicoColorSensorIOInputs {

        public byte[] buffer = new byte[257];

    }
    
}
