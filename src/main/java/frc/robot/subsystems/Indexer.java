package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.Neo;

public class Indexer extends SubsystemBase {
    private final Neo triggerWheel;
    private double desiredSpeed;

    public Indexer() {
        triggerWheel = new Neo(IntakeConstants.TRIGGER_WHEEL_CAN_ID);
        desiredSpeed = 0;
        configMotor();
    }

    public void configMotor() {
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        triggerWheel.setSmartCurrentLimit(IntakeConstants.TRIGGER_WHEEL_STALL_CURRENT_LIMIT_AMPS,
                IntakeConstants.TRIGGER_WHEEL_FREE_CURRENT_LIMIT_AMPS);
        triggerWheel.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        triggerWheel.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        triggerWheel.setInverted(false);

        // sets brake mode
        triggerWheel.setBrakeMode();
    }

    public double getDesiredSpeed() {
        return desiredSpeed;
    }

    public void setDesiredSpeed(double speed) {
        desiredSpeed = speed;
        triggerWheel.setTargetVelocity(speed);
    }

    public Command setSpeedCommand(double speed) {
        return runOnce(() -> setDesiredSpeed(speed));
    }

    public Command toShooter() {
        return setSpeedCommand(IntakeConstants.SHOOTER_TRIGGER_WHEEL_SPEED);
    }

    public Command toTrap() {
        return setSpeedCommand(IntakeConstants.TRAP_TRIGGER_WHEEL_SPEED);
    }

    public Command stop() {
        return setSpeedCommand(IntakeConstants.STOP_SPEED);
    }
}