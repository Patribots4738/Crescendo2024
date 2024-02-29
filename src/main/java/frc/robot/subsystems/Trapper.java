package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.TrapConstants;
import frc.robot.util.rev.Neo;

public class Trapper extends SubsystemBase {
    private final Neo trapper;
    private double desiredSpeed = 0;

    public Trapper() {
        trapper = new Neo(TrapConstants.TRAP_CAN_ID, true);
        trapper.setSmartCurrentLimit(TrapConstants.TRAP_CURRENT_LIMIT);
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
        if (desiredSpeed != percent) {
            desiredSpeed = percent;
            
            trapper.set(percent);
        }
    }

    public Command outtake() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_OUTTAKE_PERCENT));
    }

    public Command outtakeSlow() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_OUTTAKE_SLOW));
    }

    public Command intakeSlow() {
        return runOnce(() -> setSpeed(0.1));
    }

    public Command stopCommand() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_STOP_PERCENT));
    }

    public Command intake() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_INTAKE_PERCENT));
    }

    public Command toggleSpeed() {
        return Commands.either(
            outtake(), 
            stopCommand(), 
            () -> desiredSpeed == 0);
    }

    public boolean hasPiece() {
        return false;
    }

}
