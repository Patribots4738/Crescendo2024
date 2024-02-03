package frc.robot.subsystems.elevator.elevator;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Neo;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.TrapConstants;

public class Elevator extends SubsystemBase implements ElevatorIO{
    private final Neo elevator;


    /** Creates a new Elevator. */
    public Elevator() {
        elevator = new Neo(TrapConstants.LEFT_ELEVATOR_CAN_ID);
        configMotors();
    }

    public void configMotors() {
        elevator.setSmartCurrentLimit(TrapConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        elevator.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        elevator.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        elevator.getEncoder().setPositionConversionFactor(TrapConstants.ELEVATOR_POSITION_CONVERSION_FACTOR);
        elevator.setPID(TrapConstants.TRAP_PID);
    }

    @Override
    public void periodic() {
        RobotContainer.components3d[NTConstants.CLAW_INDEX] = new Pose3d(
            0, 0, elevator.getPosition() * TrapConstants.CLAW_POSITION_MULTIPLIER, 
            new Rotation3d()
        );
        RobotContainer.components3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
            0, 0, elevator.getPosition(),
            new Rotation3d()
        );
    }

    public double getPosition() {
        return elevator.getPosition();
    }

    public BooleanSupplier isAtTargetPosition() {
        return () -> MathUtil.applyDeadband(
                Math.abs(
                        this.getPosition() - elevator.getTargetPosition()),
                TrapConstants.ELEVATOR_DEADBAND) == 0;
    }

    public void setPosition(double pos) {
        elevator.setTargetPosition(pos);
        RobotContainer.desiredComponents3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
            0, 0, pos,
            new Rotation3d()
        );
        RobotContainer.desiredComponents3d[NTConstants.CLAW_INDEX] = new Pose3d(
            0, 0, pos*TrapConstants.CLAW_POSITION_MULTIPLIER,
            new Rotation3d()
        );
    }

    public Command setPositionCommand(double pos) {
        return runOnce(() -> this.setPosition(pos))
                .andThen(Commands.waitUntil(this.isAtTargetPosition()));
    }

    private void toTop() {
        this.setPosition(TrapConstants.TRAP_PLACE_POS);
    }

    private void toBottom() {
        this.setPosition(TrapConstants.RESET_POS);
    }

    public Command toBottomCommand() {
        return runOnce(this::toBottom);
    }

    public Command toTopCommand() {
        return runOnce(this::toTop);
    }

    public Command stop() {
        return runOnce(() -> elevator.stopMotor());
    }

}
