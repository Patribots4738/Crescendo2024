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
    private boolean hadGamePieceLast = false;
    private double startIntakingTimestamp = 0;

    /** Creates a new Claw. */
    public Claw() {
        claw = new Neo(TrapConstants.CLAW_CAN_ID);
        configMotors();
    }

    @Override
    public void periodic() {
        this.hasGamePiece = hasGamePiece();
        this.hadGamePieceLast = this.hasGamePiece;
    }

    public Command placeCommand() {
        return runOnce(this::expel)
                .andThen(
                    Commands.waitSeconds(TrapConstants.OUTTAKE_TIME)
                ).andThen(
                    runOnce(this::stop)
                ).andThen(
                    setHasGamePieceCommand(false)
                );
    }

    public Command setHasGamePieceCommand(boolean hasGamePiece) {
        return Commands.runOnce(() -> this.hasGamePiece = hasGamePiece);
    }

    public Command backToHandoffCommand() {
        return runOnce(this::back)
            .andThen(
                Commands.waitSeconds(TrapConstants.INTAKE_TIME))
            .andThen(
                runOnce(this::stop));
    }

    public boolean hasGamePiece() {
        // needs logic
        
        // if appliedoutput <= desired/2 and 
        // current this loop and last loop > constant
        // desired speed > constant 
        // we've been intaking over .25 seconds (make constant);
        // if (hasGamePiece) {
        //     this.hasGamePiece = claw.getTargetVelocity() > 0;
        // }
        // else {
        //     this.hasGamePiece = 
        //         (-0.25 < claw.getAppliedOutput() && claw.getAppliedOutput() < 0) && 
        //         (current > 15 && claw.getOutputCurrent() > 15) &&
        //         (desiredSpeed > 0.45) &&
        //         (DriverUI.currentTimestamp - startedIntakingTimestamp > 0.25);
        // }
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

    public Command expel() {
        return Commands.runOnce(() -> claw.setTargetVelocity(TrapConstants.CLAW_OUTTAKE));
    }

    public Command stop() {
        return Commands.runOnce(() -> claw.setTargetVelocity(0));
    }

    public void back() {
        claw.setTargetVelocity(TrapConstants.CLAW_INTAKE);
    }

}
