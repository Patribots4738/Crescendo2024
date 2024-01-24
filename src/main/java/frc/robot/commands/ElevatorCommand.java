// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubs.Claw;
import frc.robot.subsystems.ElevatorSubs.Elevator;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Constants.TrapConstants;

// TODO: convert to placement? 
public class ElevatorCommand {
    /** Creates a new Elevator. */
    private final Elevator elevator;
    private final Claw claw;

    public ElevatorCommand(Elevator elevator, Claw claw) {

        this.elevator = elevator;
        this.claw = claw;
    }

    public Command placeTrapCommand() {
        return Commands.runOnce(
                () -> this.elevator.setPositionCommand(TrapConstants.TRAP_PLACE_POS)).andThen(
                        Commands.waitUntil(elevator.isAtTargetPosition()))
                .andThen(claw.placeCommand());
    }

    public Command intakeFromHandoff() {
        return (claw.intakeFromHandoff());
    }

    public void setTargetPosition(double pos) {
        this.elevator.setPosition(pos);
    }

    public Command outtake() {
        return this.claw.outtake();
    }

    public Command intake() {
        return this.claw.intake();
    }

    public Command stopClaw() {
        return this.claw.stop();
    }

    public Command stopMotors() {
        return Commands.parallel(
                elevator.stop(),
                claw.stop());
    }
}
