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
import monologue.Logged;
import monologue.Annotations.Log;

public class Elevator extends SubsystemBase implements Logged {
    private final Neo elevator;
    @Log
    private double pos = 0, desiredPos = 0;

    @Log
    private boolean atDesiredPos = false;

    /** Creates a new Elevator. */
    public Elevator() {
        elevator = new Neo(TrapConstants.ELEVATOR_CAN_ID);
        configMotors();
    }

    public void configMotors() {
        elevator.setSmartCurrentLimit(TrapConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        elevator.setPositionConversionFactor(TrapConstants.ELEVATOR_POSITION_CONVERSION_FACTOR);
        elevator.setPID(TrapConstants.TRAP_PID);

        // Change to brake when done testing
        elevator.setCoastMode();
    }

    @Override
    public void periodic() {
        pos = elevator.getPosition();
        desiredPos = elevator.getTargetPosition();

        atDesiredPos = atDesiredPosition();

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
                .andThen(Commands.waitUntil(this::atDesiredPosition));
    }

    public Command toBottomCommand() {
        return setPositionCommand(TrapConstants.RESET_POS);
    }

    public Command toTopCommand() {
        return setPositionCommand(TrapConstants.TRAP_PLACE_POS);
    }

    public Command indexCommand() {
        return setPositionCommand(TrapConstants.INDEX_POS);
    }

    public Command stopCommand() {
        return runOnce(() -> elevator.stopMotor());
    }

    public boolean atDesiredPosition() {
		return MathUtil.applyDeadband(pos - desiredPos, TrapConstants.ELEVATOR_DEADBAND) == 0;
	}
}
