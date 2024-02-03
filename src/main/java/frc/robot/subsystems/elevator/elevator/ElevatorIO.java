package frc.robot.subsystems.elevator.elevator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public interface ElevatorIO {

    public double getPosition();

    public BooleanSupplier isAtTargetPosition();

    public Command toBottomCommand();
    public Command toTopCommand();
    
    public Command stop();
    
    public void setPosition(double pos);
    
    public Command setPositionCommand(double pos);
    
}
