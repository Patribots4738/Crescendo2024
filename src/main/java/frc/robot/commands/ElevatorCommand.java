// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubs.Elevator;

public class ElevatorCommand extends Command {
  /** Creates a new Elevator. */
  private final Elevator elevator;
  double targetPosition;

  //TODO: this need to be modified to control the Claw and ELevator, not just the elevator to move up and down.
  public ElevatorCommand (Elevator elevator, double targetPosition) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.elevator.setPosition(this.targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isAtTargetPosition().getAsBoolean();
  }
}
