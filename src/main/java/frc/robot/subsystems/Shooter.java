// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;
import frc.robot.util.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */
  private final Neo motorLeft;
  private final Neo motorRight;

  public Shooter() {

    motorLeft = new Neo(ShooterConstants.LEFT_SHOOTER_CAN_ID);
    motorRight = new Neo(ShooterConstants.RIGHT_SHOOTER_CAN_ID);

    configMotors();

  }

  public void configMotors() {
      motorLeft.setPID(
        ShooterConstants.SHOOTER_P,
        ShooterConstants.SHOOTER_I,
        ShooterConstants.SHOOTER_D,
        ShooterConstants.SHOOTER_MIN_OUTPUT,
        ShooterConstants.SHOOTER_MAX_OUTPUT
      );

      motorLeft.addFollower(motorRight, true);
      // Current limit
      // Soft stops (dont extend too high)
      // Position conversion factor (make gear ratio in constants)
      // 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public Command shoot(Pose2d distanceToTarget, boolean shootingAtSpeaker) {
    
    return runOnce(() -> motorLeft.setTargetVelocity(1));
  }
}
