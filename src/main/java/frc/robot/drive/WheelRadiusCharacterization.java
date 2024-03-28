// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;
import monologue.Logged;
import monologue.Annotations.Log;
import java.util.function.DoubleSupplier;

/**
 * This command is used to characterize the wheel radius of the swerve drive.
 * What that means is that the robot is able to calculate it's own wheel radius of each wheel
 * by driving in a circle and measuring the distance traveled by each wheel.
 * 
 * The wheel radius is important for the swerve drive to be able to accurately drive in a straight line.
 * Because of the wear and tear on the wheels, the radius of the wheel can change over time.
 * 
 * This command is used to calculate the wheel radius of the swerve drive and is "inspired" by 6328
 */
public class WheelRadiusCharacterization extends Command implements Logged {
    // wheel radius (meters) =
    // gyro delta (radians) * drive base radius (meters) / wheel position delta
    // (radians)
    @Log
    private double gyroDelta = 0.0, wheelPosDelta = 0.0, currentEffectiveWheelRadius = 0.0;

    @Log
    private double lastGyroYawRads = 0.0, accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private Swerve swerve;

    private final DoubleSupplier gyroYawRadsSupplier = () -> swerve.getGyroRotation2d().getRadians();

    // wheel radius (meters) =
    // gyro delta (radians) * drive base radius (meters) / wheel position delta
    // (radians)
    public WheelRadiusCharacterization(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(this.swerve);
        // Use addRequirements() here to declare subsystem dependencies.e
    }

    @Override
    public void initialize() {
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = swerve.getWheelRadiusCharacterizationPosition();

    }

    @Override
    public void execute() {
        swerve.drive(new ChassisSpeeds(0.0, 0.0, .5));
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositions = swerve.getWheelRadiusCharacterizationPosition();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
        }

        averageWheelPosition /= 4.0;
        System.out.println(averageWheelPosition);
        double driveRadius = Math.hypot(DriveConstants.WHEEL_BASE, DriveConstants.TRACK_WIDTH) / 2.0;
        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    }

    @Override
    public void end(boolean interrupted) {
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("\nNot enough data for characterization\n" + accumGyroYawRads);
        } else {
            System.out.println(
                    "\nEffective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches\n");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return accumGyroYawRads > Math.PI * 2;
    }
}
