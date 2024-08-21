package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Gyro implements GyroIO {
    private final Pigeon2 pigeon;
    public static final int id = 0;
    private final GyroIOInputsAutoLogged inputs;
    
    @AutoLogOutput(key = "Gyro/Rotation2d")
    private Rotation2d gyroRotation2d;

    public Gyro(GyroIOInputsAutoLogged inputs) {
        pigeon = new Pigeon2(id);
        pigeon.optimizeBusUtilization();
        this.inputs = inputs;
    }


    public Rotation2d getYaw() {
        return inputs.yawPosition;
    }

    public double getYawVelocity() {
        return inputs.yawVelocityRadsPerSec;
    }

    public void setYaw(double value) {
        pigeon.setYaw(value);
    }

    public double getPitch() {
        return pigeon.getPitch().refresh().getValueAsDouble();
    }
    public double getRoll() {
        return pigeon.getRoll().refresh().getValueAsDouble();
    }


    @Override
    public void updateInputs() {
        inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw().refresh().getValueAsDouble());
        inputs.yawVelocityRadsPerSec = 
            Units.degreesToRadians(pigeon.getAngularVelocityZWorld().refresh().getValueAsDouble());
    }
}
