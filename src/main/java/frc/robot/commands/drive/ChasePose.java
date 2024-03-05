// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.AutoConstants;

public class ChasePose extends Command {
    private Swerve swerve;
    private static Pose2d desiredPose = new Pose2d();

    /** Creates a new ChasePose. */
    public ChasePose(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerve.resetHDC();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        ChassisSpeeds desiredSpeeds = AutoConstants.HDC.calculate(
                swerve.getPose(),
                desiredPose,
                0,
                desiredPose.getRotation());

        swerve.setDesiredPose(desiredPose);

        swerve.drive(desiredSpeeds);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return swerve.atHDCPose();
    }

    public static void updateDesiredPose(Pose2d newPose) {
        desiredPose = newPose;
    }
}
