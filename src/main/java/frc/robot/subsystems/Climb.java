package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Neo;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.FieldConstants.ChainPosition;
import monologue.Logged;

public class Climb extends SubsystemBase implements Logged {

    private final Neo leftMotor;
    private final Neo rightMotor;

    public Climb() {
        leftMotor = new Neo(ClimbConstants.LEFT_CLIMB_CAN_ID);
        rightMotor = new Neo(ClimbConstants.RIGHT_CLIMB_CAN_ID);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        RobotContainer.components3d[NTConstants.LEFT_CLIMB_INDEX] = new Pose3d(
            0, leftMotor.getPosition(), 0,
            new Rotation3d()
        );
        RobotContainer.components3d[NTConstants.RIGHT_CLIMB_INDEX] = new Pose3d(
            0, rightMotor.getPosition(), 0,
            new Rotation3d()
        );
    }

    public void setPosition(double pos1, double pos2) {
        leftMotor.setPosition(pos1);
        rightMotor.setPosition(pos2);
        RobotContainer.desiredComponents3d[NTConstants.LEFT_CLIMB_INDEX] = new Pose3d(
            0, pos1, 0,
            new Rotation3d()
        );
        RobotContainer.desiredComponents3d[NTConstants.RIGHT_CLIMB_INDEX] = new Pose3d(
            0, pos2, 0,
            new Rotation3d()
        );
    }

    public Command toTop(ChainPosition chainPosition) {

        if (chainPosition == ChainPosition.LEFT) {
            return runOnce(() -> this.setPosition(ClimbConstants.ALMOST_HIGH_LIMIT, ClimbConstants.HIGH_LIMIT));
        } else if (chainPosition == ChainPosition.RIGHT) {
            return runOnce(() -> this.setPosition(ClimbConstants.HIGH_LIMIT, ClimbConstants.ALMOST_HIGH_LIMIT));
        } else {
            return runOnce(() -> this.setPosition(ClimbConstants.HIGH_LIMIT, ClimbConstants.HIGH_LIMIT));
        }

    }

    public Command toBottom() {

        // This mimics the behavior of an asProxy command
        // without the inclusion of the wrapper that asProxy brings along with it.
        // therefore it is more advantagous to use this
        // becuase we can't have two super.runOnce's
        // as they would require this subsystem twice
        // this is very bad and need fix :D
        ParallelRaceGroup group = new ParallelRaceGroup();
        group.addCommands(Commands.runOnce(() -> rightMotor.setPosition(ClimbConstants.ROCK_BOTTOM)));
        group.addCommands(Commands.runOnce(() -> leftMotor.setPosition(ClimbConstants.ROCK_BOTTOM)));
        group.addRequirements(this);

        return group.andThen();
    }

}
