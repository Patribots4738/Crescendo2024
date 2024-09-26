package frc.robot.subsystems.climb;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.rev.Neo;

public class Climb extends SubsystemBase implements ClimbIO {

    private final Neo leftMotor;
    private final Neo rightMotor;

    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    @AutoLogOutput(key = "Subsystems/Climb/AtDesiredPosition")
    private boolean atDesiredPos = false;
    
    @AutoLogOutput(key = "Subsystems/Climb/HooksUp")
    private boolean hooksUp = false;

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

        updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Climb", inputs);

        atDesiredPos = (
            MathUtil.applyDeadband(
				Math.abs(
						inputs.leftPositionMeters - inputs.leftTargetPositionMeters),
				ClimbConstants.CLIMB_DEADBAND) == 0 &&
            MathUtil.applyDeadband(
				Math.abs(
						inputs.rightPositionMeters - inputs.rightTargetPositionMeters),
				ClimbConstants.CLIMB_DEADBAND) == 0);
        hooksUp = getHooksUp();

        RobotContainer.components3d[NTConstants.LEFT_CLIMB_INDEX] = new Pose3d(
            0, 0, inputs.leftPositionMeters,
            new Rotation3d()
        );
        RobotContainer.components3d[NTConstants.RIGHT_CLIMB_INDEX] = new Pose3d(
            0, 0, inputs.rightPositionMeters,
            new Rotation3d()
        );
    }

    public void setPosition(Pair<Double, Double> positionPair) {
        setPosition(positionPair.getFirst(), positionPair.getSecond());
    }

    public void setPosition(double pos) {
        setPosition(pos, pos);
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

        if (inputs.leftTargetPositionMeters != pos1) {
            leftMotor.setTargetPosition(pos1);
        }

        if (inputs.rightTargetPositionMeters != pos2) {
            rightMotor.setTargetPosition(pos2);
        }

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
        setPosition(ClimbConstants.TOP_LIMIT);
    }

    public void toBottom() {
        setPosition(ClimbConstants.BOTTOM_LIMIT);
    }

    public Command toTopCommand() {
        return runOnce(() -> setPosition(ClimbConstants.TOP_LIMIT));
    }

    public Command toMiddleCommand() {
        return runOnce(() -> setPosition(ClimbConstants.TOP_LIMIT * 4.0/7.0));
    }

    public Command toBottomCommand() {
        return runOnce(() -> setPosition(ClimbConstants.BOTTOM_LIMIT));
    }

    public Command povUpCommand() {
        return Commands.either(
            toMiddleCommand(), 
            toTopCommand(), 
            () -> inputs.leftTargetPositionMeters == ClimbConstants.TOP_LIMIT || inputs.rightTargetPositionMeters == ClimbConstants.BOTTOM_LIMIT);
    }

    public Command toggleCommand() {
        return Commands.either(
            toBottomCommand(), 
            toTopCommand(),
            () -> inputs.leftTargetPositionMeters == ClimbConstants.TOP_LIMIT);
    }

    public BooleanSupplier atDesiredPosition() {
		return () -> (
            MathUtil.applyDeadband(
				Math.abs(
						inputs.leftPositionMeters - inputs.leftTargetPositionMeters),
				ClimbConstants.CLIMB_DEADBAND) == 0 &&
            MathUtil.applyDeadband(
				Math.abs(
						inputs.rightPositionMeters - inputs.rightTargetPositionMeters),
				ClimbConstants.CLIMB_DEADBAND) == 0);
	}

    public boolean getHooksUp() {
        return (inputs.leftTargetPositionMeters > 0 || inputs.rightTargetPositionMeters > 0);
    }

    @Override
    public void updateInputs(ClimbIOInputsAutoLogged inputs) {

        inputs.leftPositionMeters = leftMotor.getPosition();
        inputs.leftTargetPositionMeters = leftMotor.getTargetPosition();
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput();
        inputs.leftOutputCurrentAmps = leftMotor.getOutputCurrent();

        inputs.rightPositionMeters = rightMotor.getPosition();
        inputs.rightTargetPositionMeters = rightMotor.getTargetPosition();
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput();
        inputs.rightOutputCurrentAmps = rightMotor.getOutputCurrent();

    }
}
