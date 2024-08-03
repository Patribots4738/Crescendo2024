package frc.robot.subsystems.ampper;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.rev.Neo;

public class Ampper extends SubsystemBase implements AmpperIO {
    private final Neo motor;
    private final AmpperIOInputsAutoLogged inputs = new AmpperIOInputsAutoLogged();

    @AutoLogOutput(key = "Ampper/Desired")
    private double desiredPercent = 0;

    public Ampper() {
        motor = new Neo(ElevatorConstants.AMPPER_CAN_ID, false, ElevatorConstants.AMPPER_INVERTION);
        motor.setSmartCurrentLimit(ElevatorConstants.AMPPER_CURRENT_LIMIT);
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
    }
 
    public Command intakeFromHandoff() {
        return intake()
            .andThen(Commands.waitSeconds(ElevatorConstants.INTAKE_TIME))
            .andThen(stopCommand());
    }

    public void setPercent(double percent) {
        percent = 
            MathUtil.clamp(
                percent, 
                ElevatorConstants.AMPPER_LOWER_PERCENT_LIMIT, 
                ElevatorConstants.AMPPER_UPPER_PERCENT_LIMIT);
        if (desiredPercent != percent) {
            desiredPercent = percent;
            
            motor.set(percent);
        }
    }

    public Command outtake() {
        return runOnce(() -> setPercent(ElevatorConstants.AMPPER_OUTTAKE_PERCENT));
    }

    public Command outtakeSlow() {
        return runOnce(() -> setPercent(ElevatorConstants.AMPPER_OUTTAKE_SLOW));
    }

    public Command outtakeSlow(double seconds) {
        return 
            Commands.sequence(
                outtakeSlow(),
                Commands.waitSeconds(seconds),
                stopCommand());
    }

    public Command intakeSlow() {
        return runOnce(() -> setPercent(0.4));
    }

    public Command intakeSlow(double speed) {
        return runOnce(() -> setPercent(speed));
    }

    public Command stopCommand() {
        return runOnce(() -> setPercent(ElevatorConstants.AMPPER_STOP_PERCENT));
    }

    public Command intake() {
        return runOnce(() -> setPercent(ElevatorConstants.AMPPER_INTAKE_PERCENT));
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
            () -> desiredPercent == 0);
    }

    public Command setPercentCommand(double percent) {
        return runOnce(() -> setPercent(percent));
    }

    public Command setCoastMode() {
        return Commands.runOnce(() -> motor.setCoastMode()).ignoringDisable(true);
    }

    public Command setBrakeMode() {
        return Commands.runOnce(() -> motor.setBrakeMode()).ignoringDisable(true);
    }

    public void updateInputs(AmpperIOInputs inputs) {
        inputs.positionRotations = motor.getPosition();
        inputs.velocityRPM = motor.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput();
    }
}
