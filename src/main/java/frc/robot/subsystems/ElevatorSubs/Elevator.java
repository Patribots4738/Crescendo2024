// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorSubs;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.TrapConstants;
import frc.robot.util.Constants.FieldConstants.ChainPosition;
import monologue.Logged;

public class Elevator extends SubsystemBase {
    private final Neo leftElevator;
    private final Neo rightElevator;

    /** Creates a new Elevator. */
    public Elevator() {
        leftElevator = new Neo(TrapConstants.LEFT_ELEVATOR_CAN_ID);
        rightElevator = new Neo(TrapConstants.RIGHT_ELEVATOR_CAN_ID);
        configMotors();
    }

    public void configMotors() {
        leftElevator.setSmartCurrentLimit(TrapConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        leftElevator.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        leftElevator.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        rightElevator.setSmartCurrentLimit(TrapConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        rightElevator.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        rightElevator.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        leftElevator.addFollower(rightElevator, true);

        leftElevator.setPID(
            TrapConstants.TRAP_P,
            TrapConstants.TRAP_I,
            TrapConstants.TRAP_D, 
            0, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getPosition() {
        return leftElevator.getPosition();
    }

    public boolean isAtTargetPosition() {
        return MathUtil.applyDeadband(Math.abs(this.getPosition() - leftElevator.getTargetPosition()),
                TrapConstants.ELEVATOR_DEADBAND) == 0;
    }

    public void setPosition(double pos) {
        leftElevator.setTargetPosition(pos);
    }

}
