// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.DriveConstants;
import monologue.Logged;
import monologue.Annotations.Log;
import java.util.function.DoubleSupplier;

// thanks 6328 :D <3
public class WheelRadiusCharacterization extends Command implements Logged{
   // wheel radius (meters) =
  //   gyro delta (radians) * drive base radius (meters) / wheel position delta (radians)	
  @Log
  private double gyroDelta = 0.0, wheelPosDelta = 0.0, currentEffectiveWheelRadius = 0.0;

  @Log
  private double lastGyroYawRads = 0.0, accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private Swerve swerve; 

  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);
  private double robotTimestamp;
  private final DoubleSupplier gyroYawRadsSupplier =
  () -> swerve.getPose().getRotation().getRadians();
  // wheel radius (meters) =
  //   gyro delta (radians) * drive base radius (meters) / wheel position delta (radians)		
  public WheelRadiusCharacterization(Swerve swerve) {
    this.swerve = swerve;   
    addRequirements(this.swerve);
    // Use addRequirements() here to declare subsystem dependencies.e
  }


  @Override
  public void initialize() {
    // reset
    robotTimestamp = Robot.currentTimestamp;
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = swerve.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
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
    System.out.println(averageWheelPosition);
    averageWheelPosition /= 4.0;
    double driveRadius = Math.hypot(DriveConstants.WHEEL_BASE, DriveConstants.TRACK_WIDTH)/2.0;
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
    return Robot.currentTimestamp - robotTimestamp >= 20;
  }
}
