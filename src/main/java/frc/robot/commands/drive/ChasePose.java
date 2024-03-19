// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.AutoConstants;

public class ChasePose extends Command {
    private Swerve swerve;
    private Supplier<Pose2d> desiredPoseSupplier;
    private BooleanSupplier cancelSupplier;

    /** Creates a new ChasePose. */
    public ChasePose(Swerve swerve, Supplier<Pose2d> poseSupplier, BooleanSupplier cancelSupplier) {
        this.swerve = swerve;
        this.desiredPoseSupplier = poseSupplier;
        this.cancelSupplier = cancelSupplier;
        addRequirements(swerve);
    }

    /** Creates a new ChasePose. */
    public ChasePose(Swerve swerve, Supplier<Pose2d> poseSupplier) {
        this(swerve, poseSupplier, () -> false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerve.resetHDC();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Pose2d desiredPose2d = desiredPoseSupplier.get();

        ChassisSpeeds desiredSpeeds = AutoConstants.HDC.calculate(
                swerve.getPose(),
                desiredPose2d,
                0,
                desiredPose2d.getRotation()).div(3.0);

        swerve.setDesiredPose(desiredPose2d);

        swerve.drive(desiredSpeeds);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.stopDriving();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return swerve.atHDCPose() || cancelSupplier.getAsBoolean();
    }
}
