package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Neo;
import java.util.Optional;

public class Indexer extends SubsystemBase {
    private final Neo triggerWheel;
    private Optional<Boolean> whichWay;
    private BooleanSupplier hasPiece;

    public Indexer() {
        triggerWheel = new Neo(IntakeConstants.TRIGGER_WHEEL_CAN_ID);
        whichWay = Optional.empty();
        hasPiece = () -> false; 
        configMotor();
    }

    public void configMotor() {
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces      
        triggerWheel.setSmartCurrentLimit(IntakeConstants.TRIGGER_WHEEL_STALL_CURRENT_LIMIT_AMPS, IntakeConstants.TRIGGER_WHEEL_FREE_CURRENT_LIMIT_AMPS);
        triggerWheel.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        triggerWheel.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        triggerWheel.setInverted(false);

        //sets brake mode
        triggerWheel.setBrakeMode();
    }    

    public Optional<Boolean> whichWay(){ 
        return whichWay;
    }

    public Command toShooter() {
        whichWay = Optional.ofNullable(true);
        return runOnce(() -> triggerWheel.set(IntakeConstants.SHOOTER_TRIGGER_WHEEL_SPEED));
    }

    public Command toTrap() {
        whichWay = Optional.ofNullable(false);
        return runOnce(() -> triggerWheel.set(IntakeConstants.TRAP_TRIGGER_WHEEL_SPEED));
    }
        
    public Command stopCommand() {
        whichWay = Optional.empty();
        return runOnce(() -> triggerWheel.set(IntakeConstants.STOP_SPEED));
    }
    
    public BooleanSupplier hasPiece() {
        return this.hasPiece;
    }

    public void setHasPiece(boolean hasPiece) {
        this.hasPiece = () -> hasPiece;
    }
}