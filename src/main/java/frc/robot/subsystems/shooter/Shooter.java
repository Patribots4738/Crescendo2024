// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        motorLeft.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        motorRight.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        motorLeft.setPID(
                ShooterConstants.SHOOTER_P,
                ShooterConstants.SHOOTER_I,
                ShooterConstants.SHOOTER_D,
                ShooterConstants.SHOOTER_MIN_OUTPUT,
                ShooterConstants.SHOOTER_MAX_OUTPUT);

        motorLeft.addFollower(motorRight, true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setSpeed(double speed) {
        motorLeft.setTargetVelocity(speed);
    }

    public Command stop() {
        return Commands.run(() -> motorLeft.setTargetVelocity(0));
    }

    //TODO: Implement a way to get the RPM of the shooter
    public boolean atDesiredRPM() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'atDesiredRPM'");
    }
}
