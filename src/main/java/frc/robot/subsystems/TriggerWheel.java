package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Neo;
import java.util.Optional;
import frc.robot.util.Constants.TriggerWheelConstants;

public class TriggerWheel extends SubsystemBase {
    private final Neo triggerWheel;
    private Optional<boolean> whichWay;
    private BooleanSupplier hasPiece;

    public TriggerWheel() {
        TriggerWheel = new Neo(TriggerWheelConstants.TRIGGER_WHEEL_CAN_ID);
        whichWay = Optional.empty();
        hasPiece = () -> false; 
        configMotor();
    }

    public void configMotor() {
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces      
        triggerWheel.setSmartCurrentLimit(TriggerWheelConstants.TRIGGERWHEEL_STALL_CURRENT_LIMIT_AMPS, TriggerWheelConstants.TRIGGERWHEEL_FREE_CURRENT_LIMIT_AMPS);
        triggerWheel.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        triggerWheel.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        triggerWheel.setInverted(false);

        //sets brake mode
        triggerWheel.setBrakeMode();
    }    

    public Optional<boolean> whichWay(){ 
        return whichWay;
    }

    public Command toShooter() {
        whichWay = Optional.ofNullable(true);
        return runOnce(() -> triggerWheel.set(TriggerWheelConstants.SHOOTER_TRIGGER_WHEEL_SPEED));
    }

    public Command toTrap() {
        whichWay = Optional.ofNullable(false);
        return runOnce(() -> triggerWheel.set(TriggerWheelConstants.TRAP_TRIGGER_WHEEL_SPEED));
    }
        
    public Command stopCommand() {
        whichWay = Optional.empty();
        return runOnce(() -> triggerWheel.set(TriggerWheelConstants.STOP_SPEED));
    }
    
    public BooleanSupplier hasPiece() {
        return hasPiece;
    }

    public void setHasPiece(boolean hasPiece) {
        hasPiece = () -> hasPiece;
    }
}