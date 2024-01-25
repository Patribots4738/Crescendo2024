package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

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
    
    /**
     * The function takes an angle in degrees and converts it into a position 
     * for the pivot motor to rotate to
     * 
     * @param double The angle to set the shooter to
    */
    public void setAngle(double angle) {
        pivot.setTargetPosition(
            /**
             * TODO: Make gear ratio constant and put it here
             */
            (angle / ShooterConstants.PIVOT_MAX_ANGLE)
        );
    }

    /**
     * The function takes an angle in degrees and returns a command that sets 
     * the pivot to the angle converted to a position
     * 
     * @param double The angle to set the shooter to
     * 
     * @return The method is returning a Command object.
    */
    public Command setAngleCommand(double angle) {
        return Commands.runOnce(() -> setAngle(angle));
    }

    /**
     * The function sets the pivot angle to the rest angle constant
    */
    public void setRestAngle() {
        this.setAngle(ShooterConstants.PIVOT_REST_ANGLE);
    }

    /**
     * The function is a command that sets the rotation of the pivot to
     * a default resting position
     * 
     * @return The method is returning a Command object.
    */
    public Command setRestAngleCommand() {
        return setAngleCommand(ShooterConstants.PIVOT_REST_ANGLE);
    }

    /**
     * Determines if the pivot rotation is at its target with a small
     * tolerance
     * 
     * @return The method is returning a BooleanSupplier that returns true
     * if the pivot is at its target rotation and false otherwise
    */
    public BooleanSupplier atDesiredAngle() {
        return () ->
            (MathUtil.applyDeadband(
                Math.abs(
                    pivot.getPosition() - pivot.getTargetPosition()),
                ShooterConstants.PIVOT_DEADBAND) == 0);
    }

    /**
     * The function is a command that stops the motor
     * 
     * @return The method is returning a Command object.
    */
    public Command stop() {
        return runOnce(() -> pivot.stopMotor());
    }
}