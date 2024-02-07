package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Neo;
import frc.robot.util.PoseCalculations;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Neo.TelemetryPreference;
import monologue.Logged;

public class Climb extends SubsystemBase implements Logged {

    private final Neo leftMotor;
    private final Neo rightMotor;

    public Climb() {
        leftMotor = new Neo(ClimbConstants.LEFT_CLIMB_CAN_ID);
        rightMotor = new Neo(ClimbConstants.RIGHT_CLIMB_CAN_ID);

        configureMotors();
    }

    private void configureMotors() {
        leftMotor.setTelemetryPreference(TelemetryPreference.ONLY_RELATIVE_ENCODER);
        rightMotor.setTelemetryPreference(TelemetryPreference.ONLY_RELATIVE_ENCODER);

        leftMotor.setPositionConversionFactor(ClimbConstants.CLIMB_POSITION_CONVERSION_FACTOR);
        rightMotor.setPositionConversionFactor(ClimbConstants.CLIMB_POSITION_CONVERSION_FACTOR);

        leftMotor.setSmartCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT);

        leftMotor.setPID(ClimbConstants.CLIMB_PID);
        rightMotor.setPID(ClimbConstants.CLIMB_PID);
    }

    @Override
    public void periodic() {
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
        leftMotor.setTargetPosition(
            MathUtil.clamp(
                pos1,
                ClimbConstants.BOTTOM_LIMIT,
                ClimbConstants.TOP_LIMIT));
        rightMotor.setTargetPosition(
            MathUtil.clamp(
                pos2,
                ClimbConstants.BOTTOM_LIMIT,
                ClimbConstants.TOP_LIMIT));

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
}
