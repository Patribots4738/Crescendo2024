package frc.robot.subsystems.climb;

import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Neo;

public interface ClimbIO {
    public final int LEFT_MOTOR_CAN_ID = 0;
    public final int RIGHT_MOTOR_CAN_ID = 1;

    public Neo leftMotor = new Neo(LEFT_MOTOR_CAN_ID);
    public Neo rightMotor = new Neo(RIGHT_MOTOR_CAN_ID);

    public void setPosition(Pair<Double, Double> positionPair);
    public void setPosition(double pos1, double pos2);

    public void toTop();
    public void toBottom();

    public Command toTopCommand();
    public Command toBottomCommand();

    public Command povUpCommand(Supplier<Pose2d> positionSupplier);

}
