package frc.robot.util.custom;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public ChassisSpeeds chassisSpeeds;
    public SwerveModuleState[] moduleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.chassisSpeeds = chassisSpeeds;
        this.moduleStates = initialStates;
    }

    @Override
    public String toString() {
        String ret = chassisSpeeds.toString() + "\n";
        for (int i = 0; i < moduleStates.length; ++i ) {
            ret += "  " + moduleStates[i].toString() + "\n";
        }
        return ret;
    }
}

