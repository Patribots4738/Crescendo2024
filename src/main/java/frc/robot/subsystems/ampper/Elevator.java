package frc.robot.subsystems.ampper;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.rev.Neo;
import frc.robot.Robot;


public class Elevator extends SubsystemBase implements ElevatorIO {
    
    private final Neo motor;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    private double hitGuillotineTimestamp = 0;

    @AutoLogOutput(key = "Elevator/DesiredOverridePosition")
    private double desiredOverridePos = 0;

    @AutoLogOutput(key = "Elevator/OverrideMode")
    private boolean overrideMode = false;

    @AutoLogOutput(key = "Elevator/AtDesiredPosition")
    private boolean atDesiredPos = false;
    
    private boolean stuckOnGuillotine = false;

    /** Creates a new Elevator. */
    public Elevator() {
        motor = new Neo(ElevatorConstants.ELEVATOR_CAN_ID, false, true);
        configMotors();
    }

    public void configMotors() {
        motor.setPositionConversionFactor(ElevatorConstants.ELEVATOR_POSITION_CONVERSION_FACTOR);
        motor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_MOTOR_CURRENT_LIMIT);
        motor.setPID(ElevatorConstants.ELEVATOR_PID);
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Elevator", inputs);

        atDesiredPos = atDesiredPosition();

        // A value of 0 on hitGuillotineTimestamp indicates that we are not hitting the guillotine
        // Sets hitGuillotineTimestamp only on first hit of guillotine
        // Set back to 0 if we are no longer near the guillotine / not stuck
        if (nearGuillotine() && hitGuillotineTimestamp == 0)
            hitGuillotineTimestamp = Robot.currentTimestamp;
        else if (!nearGuillotine() && hitGuillotineTimestamp != 0) 
            hitGuillotineTimestamp = 0;
        
        updateStuck();
        RobotContainer.components3d[NTConstants.AMPPER_INDEX] = new Pose3d(
            0, 0, inputs.positionRotations * ElevatorConstants.AMPPER_POSITION_MULTIPLIER, 
            new Rotation3d()
        );
        RobotContainer.components3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
            0, 0, inputs.positionRotations,
            new Rotation3d()
        );
    }

    public double getPosition() {
        return inputs.positionRotations;
    }

    public double getDesiredPosition() {
        return inputs.targetPositionMeters;
    }

    public void setPosition(double pos) {
        pos = MathUtil.clamp(
            pos,
            ElevatorConstants.ELEVATOR_BOTTOM_LIMIT,
            ElevatorConstants.ELEVATOR_TOP_LIMIT);

        if (inputs.targetPositionMeters != pos) {
            motor.setTargetPosition(pos);
    
            RobotContainer.desiredComponents3d[NTConstants.ELEVATOR_INDEX] = new Pose3d(
                0, 0, pos,
                new Rotation3d()
            );
            RobotContainer.desiredComponents3d[NTConstants.AMPPER_INDEX] = new Pose3d(
                0, 0, pos * ElevatorConstants.AMPPER_POSITION_MULTIPLIER,
                new Rotation3d()
            );
        }

    }

    public Command setPositionCommand(DoubleSupplier pos, boolean waitUntilStuck) {
        return runOnce(() -> this.setPosition(pos.getAsDouble()))
                    .andThen(Commands.waitUntil(() -> atDesiredPosition()));
    }

    public Command setPositionCommand(DoubleSupplier pos) {
        return setPositionCommand(pos, false);
    }

    public Command setPositionCommand(double pos) {
        return setPositionCommand(() -> pos, false);
    }

    public Command setPositionCommand(double pos, boolean waitUntilStuck) {
        return setPositionCommand(() -> pos, waitUntilStuck);
    }

    public Command toBottomCommand() {
        return setPositionCommand(ElevatorConstants.BOTTOM_POS);
    }

    public Command toTopCommand() {
        return setPositionCommand(ElevatorConstants.TRAP_PLACE_POS, true);
    }

    public Command toTopIshButNotFullCommand() {
        return setPositionCommand(ElevatorConstants.DEBUG_ELEVATOR_POS, true);
    }

    public Command toNoteFixCommand() {
        return setPositionCommand(ElevatorConstants.NOTE_FIX_POS, true);
    }

    public Command toIndexCommand() {
        return setPositionCommand(ElevatorConstants.INDEX_POS).andThen(toBottomCommand());
    }

    public Command toDropCommand() {
        return setPositionCommand(ElevatorConstants.DROP_POS);
    }

    public boolean atDesiredPosition() {
        if (overrideMode)
            return MathUtil.isNear(desiredOverridePos, inputs.positionRotations, ElevatorConstants.ELEVATOR_DEADBAND);
        return MathUtil.isNear(inputs.targetPositionMeters, inputs.positionRotations, ElevatorConstants.ELEVATOR_DEADBAND);
    }

    public boolean atPosition(double position) {
        return MathUtil.isNear(position, inputs.positionRotations, ElevatorConstants.ELEVATOR_DEADBAND);
    }

    public boolean nearGuillotine() {
        return atPosition(ElevatorConstants.GUILLOTONE_POS);
    }

    public boolean getStuck() {
        return stuckOnGuillotine;
    }

    public boolean isUp() {
        return inputs.targetPositionMeters > 0;
    }

    // If we are trying to get up but we have been near guillotine for more than or equal to 0.1s then we are stuck
    public void updateStuck() {
        stuckOnGuillotine =  
            hitGuillotineTimestamp != 0
            && Robot.currentTimestamp - this.hitGuillotineTimestamp >= ElevatorConstants.STUCK_TIME_SECONDS
            && inputs.targetPositionMeters > inputs.positionRotations
            && nearGuillotine()
            && inputs.outputCurrentAmps > 35.0;
    }

    public Command resetEncoder() {
        return runOnce(motor::resetEncoder).ignoringDisable(true);
    }

    public Command overrideCommand(DoubleSupplier doubleSupplier) {
        return run(() -> {
            desiredOverridePos = MathUtil.clamp(
                desiredOverridePos + doubleSupplier.getAsDouble(),
                ElevatorConstants.ELEVATOR_BOTTOM_LIMIT,
                ElevatorConstants.ELEVATOR_TOP_LIMIT
            );
            setPosition(desiredOverridePos);
        })
        .beforeStarting(Commands.runOnce(() -> overrideMode = true))
        .finallyDo(() -> overrideMode = false);
    } 

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotations = motor.getPosition();
        inputs.targetPositionMeters = motor.getTargetPosition();
        inputs.appliedVolts = motor.getAppliedOutput();
        inputs.outputCurrentAmps = motor.getOutputCurrent();
    }

}
