// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;




import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubs.Claw;
import frc.robot.subsystems.ElevatorSubs.Elevator;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Constants.TrapConstants;

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
            () -> this.elevator.setPositionCommand(TrapConstants.TRAP_PLACE_POS))
                .andThen(
                    () -> claw.placeCommand());
    }


    public Command clawToHandoffCommand() {
        return (claw.backToHandoffCommand());
    }

    public void setTargetPosition(double pos) {
        this.elevator.setPosition(pos);
    }

    public void outtake() {
        this.claw.expel();
    }

    public void intake() {
        this.claw.back();
    }

    public void stopClaw() {
        this.claw.stop();
    }

}
