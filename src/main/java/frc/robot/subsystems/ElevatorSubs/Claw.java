// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorSubs;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;
import frc.robot.util.Constants.TrapConstants;

public class Claw extends SubsystemBase {
    private final Neo claw;
    private boolean hasGamePiece = false;

    /** Creates a new Claw. */
    public Claw() {
        claw = new Neo(TrapConstants.CLAW_CAN_ID);
        configMotors();
    }

    @Override
    public void periodic() {
        this.hasGamePiece = hasGamePiece();
    }

    public Command placeCommand() {
        return runOnce(this::expel)
                .andThen(
                    Commands.waitSeconds(TrapConstants.OUTTAKE_TIME))
                .andThen(
                    runOnce(this::stop)
                );
    }


    public boolean hasGamePiece() {
        // needs logic
        return false;
    }

    public void configMotors() {
        // needs motor configs
        claw.setSmartCurrentLimit(TrapConstants.CLAW_CURRENT_LIMIT);
        claw.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        claw.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        claw.setInverted(false);
        claw.setBrakeMode();
    }

    public boolean getHasGamePiece() {
        return this.hasGamePiece;
    }

    public void expel() {
        claw.setTargetVelocity(1);
    }

    public void stop() {
        claw.setTargetVelocity(0);
    }

}
