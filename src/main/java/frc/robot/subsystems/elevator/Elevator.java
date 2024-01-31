// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;
import frc.robot.util.Constants.TrapConstants;

public class Elevator extends SubsystemBase {
    private final Neo elevator;


    /** Creates a new Elevator. */
    public Elevator() {
        elevator = new Neo(TrapConstants.LEFT_ELEVATOR_CAN_ID);
        configMotors();
    }

    public void configMotors() {
        elevator.setSmartCurrentLimit(TrapConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        elevator.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        elevator.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        elevator.getEncoder().setPositionConversionFactor(TrapConstants.ELEVATOR_POSITION_CONVERSION_FACTOR);
        elevator.setPID(
                TrapConstants.TRAP_P,
                TrapConstants.TRAP_I,
                TrapConstants.TRAP_D,
                TrapConstants.TRAP_ELEVATOR_MIN_OUTPUT, TrapConstants.TRAP_ELEVATOR_MAX_OUTPUT);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getPosition() {
        return elevator.getPosition();
    }

    public BooleanSupplier isAtTargetPosition() {
        return () -> MathUtil.applyDeadband(
                Math.abs(
                        this.getPosition() - elevator.getTargetPosition()),
                TrapConstants.ELEVATOR_DEADBAND) == 0;
    }

    public void setPosition(double pos) {
        elevator.setTargetPosition(pos);
    }

    public Command setPositionCommand(double pos) {
        return runOnce(() -> this.setPosition(pos))
                .andThen(Commands.waitUntil(this.isAtTargetPosition()));
    }

    public Command stop() {
        return runOnce(() -> elevator.stopMotor());
    }

}