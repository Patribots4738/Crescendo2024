// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
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

    /**
     * The function sets both of the motors to a targetVelocity that is
     * the speed provided
     * 
     * @param speed A double representing the speed the motors should be set to
    */
    public void setSpeed(double speed) {
        motorLeft.setTargetVelocity(speed);
    }

    /**
     * The function is a command that sets the motor speed for both motors
     * to the speed  provided
     * 
     * @param speed A double representing the speed the motors should be set to
     * 
     * @return The method is returning a Command object.
    */
    public Command setSpeedCommand(double speed) {
        return Commands.runOnce(() -> setSpeed(speed));
    }

    /**
     * The function is a command that stops both motors
     * 
     * @return The method is returning a Command object.
    */
    public Command stop() {
        return Commands.runOnce(() -> motorLeft.stopMotor());
    }

    //TODO: Implement a way to get the RPM of the shooter
    /**
     * The function is a BooleanSupplier that represents the the condition of
     * the velocity of the motor being equal to its targetVelocity
     * 
     * 
     * @return The method is returning a BooleanSupplier that returns true if 
     * the current velocity of the motors is at the target velocity with a 
     * small tolerance
    */
    public BooleanSupplier atDesiredRPM() {
        return () ->
            (MathUtil.applyDeadband(
                Math.abs(
                    motorLeft.getVelocity() - motorLeft.getTargetVelocity()),
                ShooterConstants.SHOOTER_DEADBAND) == 0);
    }
}
