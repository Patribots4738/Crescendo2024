package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Neo;
import frc.robot.util.Constants.TrapConstants;

public class Trapper extends SubsystemBase {
    private final Neo trapper;
    private boolean hasGamePiece = false;
    private double current = 0;
    private double startIntakingTimestamp = 0;

    /** Creates a new Trapper. */
    public Trapper() {
        trapper = new Neo(TrapConstants.TRAP_CAN_ID);
        configMotors();
    }

    @Override
    public void periodic() {
        this.hasGamePiece = hasGamePiece();
        updateOutputCurrent();
    }

    public Command placeCommand() {
        return outtake()
                .andThen(Commands.waitSeconds(TrapConstants.OUTTAKE_SECONDS))
                .andThen(stop());
    }

    public Command intakeFromHandoff() {
        return intake()
                .andThen(Commands.waitSeconds(TrapConstants.INTAKE_TIME))
                .andThen(stop());
    }

    public boolean hasGamePiece() {
        // if appliedoutput <= desired/2 and
        // current this loop and last loop > constant
        // desired speed > constant
        // we've been intaking over .25 seconds (make constant);
        if (hasGamePiece) {
            this.hasGamePiece = trapper.getTargetVelocity() > 0;
        } else {
            this.hasGamePiece = (TrapConstants.TRAPPER_HAS_PIECE_LOWER_LIMIT < trapper.getAppliedOutput() &&
                    trapper.getAppliedOutput() < TrapConstants.TRAPPER_HAS_PIECE_UPPER_LIMIT) &&
                    (current > TrapConstants.TRAPPER_HAS_PIECE_MIN_CURRENT &&
                            trapper.getOutputCurrent() > TrapConstants.TRAPPER_HAS_PIECE_MIN_CURRENT)
                    &&
                    (trapper.getTargetVelocity() > TrapConstants.TRAPPER_HAS_PIECE_MIN_TARGET_VELO) &&
                    (Robot.currentTimestamp - startIntakingTimestamp > TrapConstants.TRAPPER_HAS_PIECE_MIN_TIMESTAMP);
        }
        return hasGamePiece;
    }

    public void setSpeed(double percent) {
        percent = 
            MathUtil.clamp(
                percent, 
                TrapConstants.TRAPPER_LOWER_PERCENT_LIMIT, 
                TrapConstants.TRAPPER_UPPER_PERCENT_LIMIT);
        trapper.set(percent);
    }

    public void configMotors() {
        // needs motor configs
        trapper.setSmartCurrentLimit(TrapConstants.TRAP_CURRENT_LIMIT);

        trapper.setBrakeMode();
    }

    public void updateOutputCurrent() {
        current = trapper.getOutputCurrent();
    }

    public boolean getHasGamePiece() {
        return this.hasGamePiece;
    }

    public Command outtake() {
        return runOnce(() -> trapper.set(TrapConstants.TRAPPER_OUTTAKE_PERCENT));
    }

    public Command stop() {
        return runOnce(() -> trapper.set(TrapConstants.TRAPPER_STOP_PERCENT));
    }
    
    public void updateIntakingTimestamp() {
        startIntakingTimestamp = Robot.currentTimestamp;
    }

    public Command intake() {
        return runOnce(() -> updateIntakingTimestamp())
                .andThen(runOnce(() -> trapper.set(TrapConstants.TRAPPER_INTAKE_PERCENT)));
    }

}
