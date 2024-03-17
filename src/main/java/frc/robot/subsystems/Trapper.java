package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.TrapConstants;
import frc.robot.util.rev.Neo;
import monologue.Logged;
import monologue.Annotations.Log;

public class Trapper extends SubsystemBase implements Logged {
    private final Neo motor;

    @Log
    private double desiredSpeed = 0;

    public Trapper() {
        motor = new Neo(TrapConstants.TRAP_CAN_ID, false, TrapConstants.TRAP_INVERTION);
        motor.setSmartCurrentLimit(TrapConstants.TRAP_CURRENT_LIMIT);
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
            
            motor.set(percent);
        }
    }

    public Command outtake() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_OUTTAKE_PERCENT));
    }

    public Command outtakeSlow() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_OUTTAKE_SLOW));
    }

    public Command outtakeSlow(double seconds) {
        return 
            Commands.sequence(
                intakeSlow(),
                Commands.waitSeconds(seconds),
                stopCommand());
    }

    public Command intakeSlow() {
        return runOnce(() -> setSpeed(0.2));
    }

    public Command intakeSlow(double speed) {
        return runOnce(() -> setSpeed(speed));
    }

    public Command stopCommand() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_STOP_PERCENT));
    }

    public Command intake() {
        return runOnce(() -> setSpeed(TrapConstants.TRAPPER_INTAKE_PERCENT));
    }

    public Command intake(double seconds) {
        return Commands.sequence(
            intake(),
            Commands.waitSeconds(seconds),
            stopCommand()  
        );
    }

    public Command outtake(double seconds) {
        return Commands.sequence(
            outtake(),
            Commands.waitSeconds(seconds),
            stopCommand()
        );
    }

    public Command toggleSpeed() {
        return Commands.either(
            outtakeSlow(), 
            stopCommand(), 
            () -> desiredSpeed == 0);
    }

    public Command setCoastMode() {
        return Commands.runOnce(() -> motor.setCoastMode()).ignoringDisable(true);
    }

    public Command setBrakeMode() {
        return Commands.runOnce(() -> motor.setBrakeMode()).ignoringDisable(true);
    }
}
