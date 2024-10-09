package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Gyro implements GyroIO {
    private final Pigeon2 pigeon;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();


    public Gyro(int gyroCANId) {
        pigeon = new Pigeon2(gyroCANId);
    }


    public Rotation2d getYawRotation2D() {
        return inputs.yawPosition;
    }

    public double getYawRadians() {
        return inputs.yawPosition.getRadians();
    }

    public double getYawDegrees() {
        return inputs.yawPosition.getDegrees();
    }

    public double getYawVelocity() {
        return inputs.yawVelocityRadsPerSec;
    }

    public void setYaw(double value) {
        pigeon.setYaw(value);
    }
    
    // could have pitch/roll as inputs but no need realistically
    public double getPitch() {
        return pigeon.getPitch().refresh().getValueAsDouble();
    }

    public double getRoll() {
        return pigeon.getRoll().refresh().getValueAsDouble();
    }

    public void processInputs() {
        Logger.processInputs("SubsystemInputs/Gyro", inputs);
    }

    @Override
    public void updateInputs() {
        inputs.yawPosition = pigeon.getRotation2d();
        inputs.yawVelocityRadsPerSec = 
            Units.degreesToRadians(pigeon.getAngularVelocityZWorld().refresh().getValue());
    }
}
