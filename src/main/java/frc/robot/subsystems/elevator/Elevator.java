package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.constants.Constants.NTConstants;
import frc.robot.util.constants.Constants.TrapConstants;
import frc.robot.util.motors.Neo;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;


public class Elevator extends SubsystemBase implements Logged {
    private final Neo elevator;
    @Log
    private double pos = 0, desiredPos = 0, startedElevationTimestamp = 0;

    @Log
    private boolean atDesiredPos = false, stuckOnGuillotine = false;

    /** Creates a new Elevator. */
    public Elevator() {
        elevator = new Neo(TrapConstants.ELEVATOR_CAN_ID, true);
        configMotors();
    }

    public void configMotors() {
        elevator.setPositionConversionFactor(TrapConstants.ELEVATOR_POSITION_CONVERSION_FACTOR);
        elevator.setSmartCurrentLimit(TrapConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        elevator.setPID(TrapConstants.ELEVATOR_PID);
    }

    @Override
    public void periodic() {
        pos = elevator.getPosition();
        desiredPos = elevator.getTargetPosition();

        atDesiredPos = atDesiredPosition();
        stuckOnGuillotine = stuckOnGuillotine(0.6);
        RobotContainer.components3d[NTConstants.TRAPPER_INDEX] = new Pose3d(
            0, 0, pos * TrapConstants.TRAPPER_POSITION_MULTIPLIER, 
            new Rotation3d()
        );
        RobotContainer.components3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
            0, 0, pos,
            new Rotation3d()
        );
    }

    public double getPosition() {
        return elevator.getPosition();
    }

    public void setPosition(double pos) {
        pos = MathUtil.clamp(
            pos,
            TrapConstants.ELEVATOR_BOTTOM_LIMIT,
            TrapConstants.ELEVATOR_TOP_LIMIT);

        elevator.setTargetPosition(pos);

        RobotContainer.desiredComponents3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
            0, 0, pos,
            new Rotation3d()
        );
        RobotContainer.desiredComponents3d[NTConstants.TRAPPER_INDEX] = new Pose3d(
            0, 0, pos * TrapConstants.TRAPPER_POSITION_MULTIPLIER,
            new Rotation3d()
        );
    }

    public Command setPositionCommand(double pos) {
        return runOnce(() -> this.setPosition(pos))
                // Keep the subsystem required as it gets there
                .andThen(Commands.waitUntil((() -> (atDesiredPos || stuckOnGuillotine))));
    }



    public Command toBottomCommand() {
        return setPositionCommand(TrapConstants.BOTTOM_POS);
    }

    public Command toTopCommand() {
        return setPositionCommand(TrapConstants.TRAP_PLACE_POS);
    }



    public Command toAmpCommand() {
        return setPositionCommand(TrapConstants.AMP_PLACE_POS);
    }

    public Command toIndexCommand() {
        return setPositionCommand(TrapConstants.INDEX_POS).andThen(toBottomCommand());
    }

    public Command toDropCommand() {
        return setPositionCommand(TrapConstants.DROP_POS);
    }

    public boolean atDesiredPosition() {
		return MathUtil.isNear(desiredPos, pos, TrapConstants.ELEVATOR_DEADBAND);
	}

    public boolean stuckOnGuillotine(double toleranceTime) {
        return Robot.currentTimestamp - this.startedElevationTimestamp >= toleranceTime;
    }
}
