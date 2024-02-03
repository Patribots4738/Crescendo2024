package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public interface SwerveIO {
    Pose2d getPose();

    void drive(ChassisSpeeds speeds);

    void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative);

    void setWheelsX();

    Command getSetWheelsX();

    void setModuleStates(SwerveModuleState[] desiredStates);

    void resetOdometry(Pose2d pose);

    SwerveModuleState[] getModuleStates();

    double getSpeedMetersPerSecond();

    SwerveModulePosition[] getModulePositions();

    void resetEncoders();

    Command toggleSpeed();

    void setSpeedMultiplier(double speedMultiplier);

    Command setAlignemntSpeed();

    double getSpeedMultiplier();

    void setBrakeMode();

    void setCoastMode();

    Command getAutoAlignmentCommand(Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds);

    Command getDriveCommand(Supplier<ChassisSpeeds> speeds, BooleanSupplier fieldRelative);

    double getAlignmentSpeeds(Rotation2d desiredAngle);

    Command resetHDC();
}
