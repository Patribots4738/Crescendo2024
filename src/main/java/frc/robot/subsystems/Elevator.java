package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NTConstants;
import lib.rev.Neo;
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
        motor = new Neo(ElevatorConstants.ELEVATOR_CAN_ID, false, true);
        configMotors();
    }

    public void configMotors() {
        motor.setPositionConversionFactor(ElevatorConstants.ELEVATOR_POSITION_CONVERSION_FACTOR);
        motor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        motor.setPID(ElevatorConstants.ELEVATOR_PID);
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
        RobotState.components3d[NTConstants.AMPPER_INDEX] = new Pose3d(
            0, 0, position * ElevatorConstants.AMPPER_POSITION_MULTIPLIER, 
            new Rotation3d()
        );
        RobotState.components3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
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
            ElevatorConstants.ELEVATOR_BOTTOM_LIMIT,
            ElevatorConstants.ELEVATOR_TOP_LIMIT);

        if (desiredPos != pos) {
            desiredPos = pos;
            motor.setTargetPosition(pos);
    
            RobotState.desiredComponents3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
                0, 0, pos,
                new Rotation3d()
            );
            RobotState.desiredComponents3d[NTConstants.AMPPER_INDEX] = new Pose3d(
                0, 0, pos * ElevatorConstants.AMPPER_POSITION_MULTIPLIER,
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
        return setPositionCommand(ElevatorConstants.BOTTOM_POS);
    }

    public Command toTopCommand() {
        return setPositionCommand(ElevatorConstants.TRAP_PLACE_POS, true);
    }

    public Command toNoteFixCommand() {
        return setPositionCommand(ElevatorConstants.NOTE_FIX_POS, true);
    }

    public Command toIndexCommand() {
        return setPositionCommand(ElevatorConstants.INDEX_POS).andThen(toBottomCommand());
    }

    public Command toDropCommand() {
        return setPositionCommand(ElevatorConstants.DROP_POS);
    }

    public boolean atDesiredPosition() {
	return MathUtil.isNear(desiredPos, position, ElevatorConstants.ELEVATOR_DEADBAND)
        || MathUtil.isNear(desiredOverridePos, position, ElevatorConstants.ELEVATOR_DEADBAND);
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, this.position, ElevatorConstants.ELEVATOR_DEADBAND);
    }

    public boolean nearGuillotine() {
        return atPosition(ElevatorConstants.GUILLOTINE_POS);
    }

    public boolean getStuck() {
        return stuckOnGuillotine;
    }

    // If we are trying to get up but we have been near guillotine for more than or equal to 0.1s then we are stuck
    public void updateStuck() {
        stuckOnGuillotine =  
            hitGuillotineTimestamp != 0
            && Robot.currentTimestamp - this.hitGuillotineTimestamp >= ElevatorConstants.STUCK_TIME_SECONDS
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
                ElevatorConstants.ELEVATOR_BOTTOM_LIMIT,
                ElevatorConstants.ELEVATOR_TOP_LIMIT
            );
            setPosition(desiredOverridePos);
        });
    } 
}
