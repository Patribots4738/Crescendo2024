package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface MAXSwerveModuleIO {

    @AutoLog
    class MAXSwerveModuleIOInputs {

        public double drivePositionMeters = 0.0; 
        public double driveVelocityMPS = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;


        public double turnPositionRads = 0.0;
        public double turnVelocityRadsPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyCurrentAmps = 0.0;

        public SwerveModulePosition position = new SwerveModulePosition(); //
        public SwerveModuleState state = new SwerveModuleState();
    }

    default void updateInputs(MAXSwerveModuleIOInputs inputs) {
        
    }

}