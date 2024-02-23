package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.constants.Constants.TrapConstants;
import frc.robot.util.motors.Neo;

public class Trapper extends SubsystemBase {
    private final Neo trapper;

    public Trapper() {
        trapper = new Neo(TrapConstants.TRAP_CAN_ID);
        trapper.setSmartCurrentLimit(TrapConstants.TRAP_CURRENT_LIMIT);
    }

    public Command placeCommand() {
        return outtake()
            .andThen(Commands.waitSeconds(TrapConstants.OUTTAKE_SECONDS))
            .andThen(stopCommand());
    }

    public Command intakeFromHandoff() {
        return intake()
            .andThen(Commands.waitSeconds(TrapConstants.INTAKE_TIME))
            .andThen(stopCommand());
    }

    public void setSpeed(double percent) {
        percent = 
            MathUtil.clamp(
                percent, 
                TrapConstants.TRAPPER_LOWER_PERCENT_LIMIT, 
                TrapConstants.TRAPPER_UPPER_PERCENT_LIMIT);
        trapper.set(percent);
    }

    public Command outtake() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_OUTTAKE_PERCENT));
    }

    public Command stopCommand() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_STOP_PERCENT));
    }

    public Command intake() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_INTAKE_PERCENT));
    }

}
