package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.TrapConstants;
import frc.robot.util.rev.Neo;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;


public class Elevator extends SubsystemBase implements Logged {
    
    private final Neo motor;

    @Log
    private double position = 0, desiredPos = 0, hitGuillotineTimestamp = 0, desiredOverridePos = 0;

    @Log
    private boolean atDesiredPos = false, stuckOnGuillotine = false;

    /** Creates a new Elevator. */
    public Elevator() {
        motor = new Neo(TrapConstants.ELEVATOR_CAN_ID, false, true);
        configMotors();
    }

    public void configMotors() {
        motor.setPositionConversionFactor(TrapConstants.ELEVATOR_POSITION_CONVERSION_FACTOR);
        motor.setSmartCurrentLimit(TrapConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        motor.setPID(TrapConstants.ELEVATOR_PID);
    }

    @Override
    public void periodic() {
        position = motor.getPosition();
        atDesiredPos = atDesiredPosition();

        // A value of 0 on hitGuillotineTimestamp indicates that we are not hitting the guillotine
        // Sets hitGuillotineTimestamp only on first hit of guillotine
        // Set back to 0 if we are no longer near the guillotine / not stuck
        if (nearGuillotine() && hitGuillotineTimestamp == 0)
            hitGuillotineTimestamp = Robot.currentTimestamp;
        else if (!nearGuillotine() && hitGuillotineTimestamp != 0) 
            hitGuillotineTimestamp = 0;
        
        updateStuck();
        RobotContainer.components3d[NTConstants.TRAPPER_INDEX] = new Pose3d(
            0, 0, position * TrapConstants.TRAPPER_POSITION_MULTIPLIER, 
            new Rotation3d()
        );
        RobotContainer.components3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
            0, 0, position,
            new Rotation3d()
        );
    }

    public double getPosition() {
        return position;
    }

    public double getDesiredPosition() {
        return desiredPos;
    }

    public void setPosition(double pos) {
        pos = MathUtil.clamp(
            pos,
            TrapConstants.ELEVATOR_BOTTOM_LIMIT,
            TrapConstants.ELEVATOR_TOP_LIMIT);

        if (desiredPos != pos) {
            desiredPos = pos;
            motor.setTargetPosition(pos);
    
            RobotContainer.desiredComponents3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
                0, 0, pos,
                new Rotation3d()
            );
            RobotContainer.desiredComponents3d[NTConstants.TRAPPER_INDEX] = new Pose3d(
                0, 0, pos * TrapConstants.TRAPPER_POSITION_MULTIPLIER,
                new Rotation3d()
            );
        }

    }

    public Command setPositionCommand(DoubleSupplier pos, boolean waitUntilStuck) {
        return runOnce(() -> this.setPosition(pos.getAsDouble()))
                    .andThen(Commands.waitUntil(() -> atDesiredPosition()));
    }

    public Command setPositionCommand(DoubleSupplier pos) {
        return setPositionCommand(pos, false);
    }

    public Command setPositionCommand(double pos) {
        return setPositionCommand(() -> pos, false);
    }

    public Command setPositionCommand(double pos, boolean waitUntilStuck) {
        return setPositionCommand(() -> pos, waitUntilStuck);
    }

    public Command toBottomCommand() {
        return setPositionCommand(TrapConstants.BOTTOM_POS);
    }

    public Command toTopCommand() {
        return setPositionCommand(TrapConstants.TRAP_PLACE_POS, true);
    }

    public Command toNoteFixCommand() {
        return setPositionCommand(TrapConstants.NOTE_FIX_POS, true);
    }

    public Command toIndexCommand() {
        return setPositionCommand(TrapConstants.INDEX_POS).andThen(toBottomCommand());
    }

    public Command toDropCommand() {
        return setPositionCommand(TrapConstants.DROP_POS);
    }

    public boolean atDesiredPosition() {
	return MathUtil.isNear(desiredPos, position, TrapConstants.ELEVATOR_DEADBAND)
        || MathUtil.isNear(desiredOverridePos, position, TrapConstants.ELEVATOR_DEADBAND);
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, this.position, TrapConstants.ELEVATOR_DEADBAND);
    }

    public boolean nearGuillotine() {
        return atPosition(TrapConstants.GUILLOTONE_POS);
    }

    public boolean getStuck() {
        return stuckOnGuillotine;
    }

    // If we are trying to get up but we have been near guillotine for more than or equal to 0.1s then we are stuck
    public void updateStuck() {
        stuckOnGuillotine =  
            hitGuillotineTimestamp != 0
            && Robot.currentTimestamp - this.hitGuillotineTimestamp >= TrapConstants.STUCK_TIME_SECONDS
            && desiredPos > position
            && nearGuillotine()
            && motor.getOutputCurrent() > 35.0;
    }

    public Command resetEncoder() {
        return runOnce(motor::resetEncoder).ignoringDisable(true);
    }

    public Command overrideCommand(DoubleSupplier doubleSupplier) {
        return run(() -> {
            desiredOverridePos = MathUtil.clamp(
                desiredOverridePos + doubleSupplier.getAsDouble(),
                TrapConstants.ELEVATOR_BOTTOM_LIMIT,
                TrapConstants.ELEVATOR_TOP_LIMIT
            );
            setPosition(desiredOverridePos);
        });
    } 
}
