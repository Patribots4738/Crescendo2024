package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
//import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import monologue.Annotations.Log;

public class DriveHDC  extends Command {

    private final Swerve swerve;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldRelativeSupplier;
    private final BooleanSupplier shouldMirror;

    public DriveHDC (
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationsSupplier,
            BooleanSupplier fieldRelativeSupplier,
            BooleanSupplier shouldMirror) {

        this.swerve = swerve;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationsSupplier;

        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.shouldMirror = shouldMirror;

        addRequirements(swerve);
    }

    public DriveHDC(Swerve swerve, Supplier<ChassisSpeeds> speeds, BooleanSupplier fieldRelativeSupplier, BooleanSupplier shouldMirror) {

        this.swerve = swerve;

        this.xSupplier = () -> speeds.get().vxMetersPerSecond;
        this.ySupplier = () -> speeds.get().vyMetersPerSecond;
        this.rotationSupplier = () -> speeds.get().omegaRadiansPerSecond;

        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.shouldMirror = shouldMirror;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        // The driver's right is negative 
        // on the field's axis
        double y = -ySupplier.getAsDouble();
        if (shouldMirror.getAsBoolean()) {
            x *= -1;
            y *= -1;
        }

        // if (x == 0 && y == 0 && rotationSupplier.getAsDouble() == 0) {
        //     swerve.drive(new ChassisSpeeds());
        //     return;
        // }

        HolonomicDriveController HDC = AutoConstants.HDC;

        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotationSupplier.getAsDouble(), swerve.getPose().getRotation());

        // integrate the speeds to positions with exp and twist
        Pose2d desiredPose = swerve.getPose().exp(
            new Twist2d(
                desiredSpeeds.vxMetersPerSecond * DriveConstants.MAX_SPEED_METERS_PER_SECOND * .1,
                desiredSpeeds.vyMetersPerSecond * DriveConstants.MAX_SPEED_METERS_PER_SECOND * .1,
                desiredSpeeds.omegaRadiansPerSecond * DriveConstants.MAX_SPEED_METERS_PER_SECOND * .1
            )
        );

        swerve.drive(
            HDC.calculate(swerve.getPose(), desiredPose, 0, desiredPose.getRotation())
        );

        swerve.setDesriredPose(desiredPose);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
