package frc.robot.subsystems.elevator.claw;

import edu.wpi.first.wpilibj2.command.Command;

public interface ClawIO {

    public Command placeCommand();

    public Command intakeFromHandoff();
    
    public boolean hasGamePiece();
    
    public void updateOutputCurrent();
    
    public boolean getHasGamePiece();
    
    public Command outtake();
    public Command stop();
    public Command intake();

}
