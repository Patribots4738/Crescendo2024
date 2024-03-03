// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.rev;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ModuleConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class MAXSwerveModule implements Logged{
    private final Neo drivingSpark;
    private final Neo turningSpark;

    private double chassisAngularOffset = 0;

    @Log
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        drivingSpark = new Neo(drivingCANId, true);
        
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.

        turningSpark = new Neo(turningCANId, false, ModuleConstants.TURNING_ENCODER_INVERTED, true);
        this.chassisAngularOffset = chassisAngularOffset;

        resetEncoders();
        configMotors();
        desiredState.angle = new Rotation2d(turningSpark.getPosition());
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(drivingSpark.getVelocity(),
                new Rotation2d(turningSpark.getPosition() - chassisAngularOffset));
    }

    public SparkPIDController getDrivingPIDController() {
        return drivingSpark.getPIDController();
    }
    
    public SparkPIDController getTurningPIDController() {
        return turningSpark.getPIDController();
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                drivingSpark.getPosition(),
                new Rotation2d(turningSpark.getPosition() - chassisAngularOffset));
    }   


    // gets the rotations of the wheel converted to radians
    // TODO: change 3.5 into a real number from thad
    public double getDrivePositionRadians() {
        return (drivingSpark.getPosition() / (ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR * ModuleConstants.WHEEL_TO_MOTOR_ROTATIONS)) * 2 * Math.PI;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        if (!FieldConstants.IS_SIMULATION) {
            correctedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                    new Rotation2d(turningSpark.getPosition()));
        }

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivingSpark.setTargetVelocity(correctedDesiredState.speedMetersPerSecond);
        turningSpark.setTargetPosition(correctedDesiredState.angle.getRadians());

        this.desiredState = correctedDesiredState;
    }

    /**
     * Zeroes this driving motor's encoder.
     */
    public void resetEncoders() {
        drivingSpark.resetEncoder();
    }

    /**
     * Set the full module to coast mode
     */
    public void setCoastMode() {
        drivingSpark.setCoastMode();
        turningSpark.setCoastMode();
    }

    /**
     * Set the full module to brake mode
     */
    public void setBrakeMode() {
        drivingSpark.setBrakeMode();
        turningSpark.setBrakeMode();
    }

    public void configMotors() {
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        drivingSpark.setPositionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
        drivingSpark.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turningSpark.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turningSpark.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turningSpark.setPositionPIDWrappingEnabled(true);
        turningSpark.setPositionPIDWrappingBounds(
            ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
            ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        drivingSpark.setPID(ModuleConstants.DRIVING_PID);
        turningSpark.setPID(ModuleConstants.TURNING_PID);

        drivingSpark.setSmartCurrentLimit(ModuleConstants.NEO_CURRENT_LIMIT);
        turningSpark.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);
        setBrakeMode();
    }
}