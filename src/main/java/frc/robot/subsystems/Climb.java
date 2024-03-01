package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.rev.Neo;
import monologue.Logged;
import monologue.Annotations.Log;

public class Climb extends SubsystemBase implements Logged {

    private final Neo leftMotor;
    private final Neo rightMotor;

    @Log
    private double posLeft = 0, posRight = 0, targetPosRight = 0, targetPosLeft = 0;

    @Log
    private boolean atDesiredPos = false, hooksUp = false;

    public Climb() {
        leftMotor = new Neo(ClimbConstants.LEFT_CLIMB_CAN_ID, true);
        // invert right motor in real life, not in sim
        rightMotor = new Neo(ClimbConstants.RIGHT_CLIMB_CAN_ID, true, !FieldConstants.IS_SIMULATION);

        configureMotors();
    }

    private void configureMotors() {
        leftMotor.setPositionConversionFactor(ClimbConstants.CLIMB_POSITION_CONVERSION_FACTOR);
        rightMotor.setPositionConversionFactor(ClimbConstants.CLIMB_POSITION_CONVERSION_FACTOR);

        leftMotor.setSmartCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT);

        leftMotor.setPID(ClimbConstants.CLIMB_PID);
        rightMotor.setPID(ClimbConstants.CLIMB_PID);
    }

    @Override
    public void periodic() {
        targetPosLeft = leftMotor.getTargetPosition();
        targetPosRight = rightMotor.getTargetPosition();
        posLeft = leftMotor.getPosition();
        posRight = rightMotor.getPosition();

        atDesiredPos = atDesiredPosition().getAsBoolean();
        hooksUp = getHooksUp();

        RobotContainer.components3d[NTConstants.LEFT_CLIMB_INDEX] = new Pose3d(
            0, 0, leftMotor.getPosition(),
            new Rotation3d()
        );
        RobotContainer.components3d[NTConstants.RIGHT_CLIMB_INDEX] = new Pose3d(
            0, 0, rightMotor.getPosition(),
            new Rotation3d()
        );
    }

    public void setPosition(Pair<Double, Double> positionPair) {
        setPosition(positionPair.getFirst(), positionPair.getSecond());
    }

    public void setPosition(double pos1, double pos2) {
        pos1 = 
            MathUtil.clamp(
                pos1,
                ClimbConstants.BOTTOM_LIMIT,
                ClimbConstants.TOP_LIMIT);
        pos2 = 
            MathUtil.clamp(
                pos2,
                ClimbConstants.BOTTOM_LIMIT,
                ClimbConstants.TOP_LIMIT);

        if (targetPosLeft != pos1) {
            leftMotor.setTargetPosition(pos1);
        }

        if (targetPosRight != pos2)

        rightMotor.setTargetPosition(pos2);

        RobotContainer.desiredComponents3d[NTConstants.LEFT_CLIMB_INDEX] = new Pose3d(
            0, 0, pos1,
            new Rotation3d()
        );
        RobotContainer.desiredComponents3d[NTConstants.RIGHT_CLIMB_INDEX] = new Pose3d(
            0, 0, pos2,
            new Rotation3d()
        );
    }

    public void toTop() {
        setPosition(ClimbConstants.TOP_LIMIT, ClimbConstants.TOP_LIMIT);
    }

    public void toBottom() {
        setPosition(ClimbConstants.BOTTOM_LIMIT, ClimbConstants.BOTTOM_LIMIT);
    }

    public Command toTopCommand() {
        return runOnce(() -> setPosition(ClimbConstants.TOP_LIMIT, ClimbConstants.TOP_LIMIT));
    }

    public Command toBottomCommand() {
        return runOnce(() -> setPosition(ClimbConstants.BOTTOM_LIMIT, ClimbConstants.BOTTOM_LIMIT));
    }

    public Command povUpCommand(Supplier<Pose2d> positionSupplier) {
        return runEnd(() -> {
                Pair<Double, Double> positionPair = PoseCalculations.getChainIntercepts(positionSupplier.get());
                setPosition(positionPair);
            }, 
            this::toTop
        );
    }

    public BooleanSupplier atDesiredPosition() {
		return () -> (
            MathUtil.applyDeadband(
				Math.abs(
						leftMotor.getPosition() - leftMotor.getTargetPosition()),
				ClimbConstants.CLIMB_DEADBAND) == 0 &&
            MathUtil.applyDeadband(
				Math.abs(
						rightMotor.getPosition() - rightMotor.getTargetPosition()),
				ClimbConstants.CLIMB_DEADBAND) == 0);
	}

    public boolean getHooksUp() {
        return (leftMotor.getTargetPosition() > 0 || rightMotor.getTargetPosition() > 0);
    }
}