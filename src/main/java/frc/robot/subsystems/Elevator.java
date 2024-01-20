package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;
import frc.robot.util.Constants.ElevatorConstants;
import monologue.Logged;

public class Elevator extends SubsystemBase implements Logged {

    private final Neo elevatorLeft;
    private final Neo elevatorRight;
    
    public Elevator() {
        elevatorLeft = new Neo(14);
        elevatorRight = new Neo(15);

        elevatorLeft.setIdleMode(IdleMode.kBrake);
        elevatorRight.setIdleMode(IdleMode.kBrake);

        elevatorLeft.setInverted(true);
        elevatorRight.setInverted(false);

    }    

    public Command toTop(boolean right) {
        
       if (right) {
            return runOnce(() -> elevatorLeft.setPosition(ElevatorConstants.ALMOST_HIGH_LIMIT)).alongWith
                (runOnce(() -> elevatorRight.setPosition(ElevatorConstants.HIGH_LIMIT)));
       } 
       else {
        return runOnce(() -> elevatorRight.setPosition(ElevatorConstants.ALMOST_HIGH_LIMIT)).alongWith
                (runOnce(() -> elevatorLeft.setPosition(ElevatorConstants.HIGH_LIMIT)));
       }
       
    }
    
    public Command toBottom() {
        return runOnce(() -> elevatorRight.setPosition(ElevatorConstants.ROCK_BOTTOM)).raceWith
                (runOnce(() -> elevatorLeft.setPosition(ElevatorConstants.ROCK_BOTTOM)));
    }
    
}