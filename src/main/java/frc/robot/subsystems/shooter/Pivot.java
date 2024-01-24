package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;
import frc.robot.util.Constants.ShooterConstants;

public class Pivot extends SubsystemBase {
    private Neo pivot;
    
    public Pivot() {
        this.pivot = new Neo(ShooterConstants.SHOOTER_PIVOT_CAN_ID);

        configMotor();
    }


    public void configMotor() {
        pivot.setSmartCurrentLimit(ShooterConstants.PIVOT_CURRENT_LIMIT);
        pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        pivot.setInverted(false);

        pivot.setPID(
                ShooterConstants.PIVOT_P,
                ShooterConstants.PIVOT_I,
                ShooterConstants.PIVOT_D,
                ShooterConstants.PIVOT_MIN_OUTPUT,
                ShooterConstants.PIVOT_MAX_OUTPUT);


        //sets brake mode
        pivot.setBrakeMode();
    }
    
    public void setAngle(double angle) {
        pivot.setTargetPosition(angle / ShooterConstants.PIVOT_MAX_ANGLE);
    }

    public Command setAngleCommand(double angle) {
        return Commands.runOnce(() -> setAngle(angle));
    }

    public void setRestAngle() {
        this.setAngle(ShooterConstants.PIVOT_REST_ANGLE);
    }

    public Command setRestAngleCommand() {
        return setAngleCommand(ShooterConstants.PIVOT_REST_ANGLE);
    }

    public BooleanSupplier isAtDesiredAngle() {
        return () ->
            (MathUtil.applyDeadband(
                Math.abs(
                    pivot.getPosition() - pivot.getTargetPosition()),
                ShooterConstants.PIVOT_DEADBAND) == 0);
    }
}