// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ModuleConstants;
import frc.robot.util.rev.Neo;

public class MAXSwerveModule implements MAXSwerveModuleIO {
    private final Neo driveMotor;
    private final Neo turnMotor;

    private double chassisAngularOffset = 0;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private final MAXSwerveModuleIOInputsAutoLogged inputs = new MAXSwerveModuleIOInputsAutoLogged();
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        driveMotor = new Neo(drivingCANId, true);
        
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.

        turnMotor = new Neo(turningCANId, false, ModuleConstants.TURNING_ENCODER_INVERTED, true);
        this.chassisAngularOffset = chassisAngularOffset;
        resetEncoders();
        configMotors();
        desiredState.angle = new Rotation2d(turnMotor.getPosition());
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return inputs.state;
    }

    public SparkPIDController getDrivingPIDController() {
        return driveMotor.getPIDController();
    }
    
    public SparkPIDController getTurningPIDController() {
        return turnMotor.getPIDController();
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return inputs.position;
    }

    // gets the rotations of the wheel converted to radians
    // We need to undo the position conversion factor
    public double getDrivePositionRadians() {
        return inputs.drivePositionRads;
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
                    new Rotation2d(turnMotor.getPosition()));
        }

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        driveMotor.setTargetVelocity(correctedDesiredState.speedMetersPerSecond);
        turnMotor.setTargetPosition(correctedDesiredState.angle.getRadians());

        this.desiredState = correctedDesiredState;
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Zeroes this driving motor's encoder.
     */
    public void resetEncoders() {
        driveMotor.resetEncoder();
    }

    /**
     * Set the full module to coast mode
     */
    public void setCoastMode() {
        driveMotor.setCoastMode();
        turnMotor.setCoastMode();
    }

    /**
     * Set the full module to brake mode
     */
    public void setBrakeMode() {
        driveMotor.setBrakeMode();
        turnMotor.setBrakeMode();
    }

    public void configMotors() {
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveMotor.setPositionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
        driveMotor.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turnMotor.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turnMotor.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turnMotor.setPositionPIDWrappingEnabled(true);
        turnMotor.setPositionPIDWrappingBounds(
            ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
            ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        driveMotor.setPID(ModuleConstants.DRIVING_PID);
        turnMotor.setPID(ModuleConstants.TURNING_PID);

        driveMotor.setSmartCurrentLimit(ModuleConstants.NEO_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);
        setBrakeMode();
    }
    
    public void processInputs() {
        Logger.processInputs("SubsystemInputs/MAXSwerveModule", inputs);
    }
    public void updateInputs() {
        // swerve module position and state

        inputs.position = new SwerveModulePosition(
            driveMotor.getPosition(),
            new Rotation2d(turnMotor.getPosition() - chassisAngularOffset));
        
        inputs.state = new SwerveModuleState(driveMotor.getVelocity(),
            new Rotation2d(turnMotor.getPosition() - chassisAngularOffset));

        // drive motor
        inputs.drivePositionRads = (driveMotor.getPosition() / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS) * 2 * Math.PI;
        inputs.driveVelocityRotationsPerSec = driveMotor.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput();
        inputs.driveSupplyCurrentAmps = driveMotor.getOutputCurrent();


        // turning motor
        inputs.turnAppliedVolts = turnMotor.getAppliedOutput();
        inputs.turnPosition = turnMotor.getPosition();
        inputs.turnVelocityRotationsPerSec = turnMotor.getVelocity();
        inputs.turnSupplyCurrentAmps = turnMotor.getOutputCurrent();
    }
}