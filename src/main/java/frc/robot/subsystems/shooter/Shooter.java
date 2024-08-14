package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.rev.Neo;

public class Shooter extends SubsystemBase implements ShooterIO {
    /** Creates a new shooter. */
    private final Neo leftMotor;
    private final Neo rightMotor;

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    @AutoLogOutput (key = "Shooter/AtDesiredRPM")
    private boolean atDesiredRPM = false;

    @AutoLogOutput (key = "Shooter/AtDesiredPassRPM")
    private boolean atDesiredPassRPM = false;

    public Shooter() {

        leftMotor = new Neo(ShooterConstants.LEFT_SHOOTER_CAN_ID, true, true);
        rightMotor = new Neo(ShooterConstants.RIGHT_SHOOTER_CAN_ID, true);

        configMotors();
    }

    public void configMotors() {
        leftMotor.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        rightMotor.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
        leftMotor.setPID(ShooterConstants.SHOOTER_PID);

        rightMotor.setPID(ShooterConstants.SHOOTER_PID);

        leftMotor.setCoastMode();
        rightMotor.setCoastMode();
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Shooter", inputs);
        
        atDesiredRPM = 
            MathUtil.isNear(
                inputs.leftVelocityRPM, inputs.leftTargetVelocityRPM,
                ShooterConstants.SHOOTER_RPM_DEADBAND)
            && MathUtil.isNear(
                inputs.rightVelocityRPM, inputs.rightTargetVelocityRPM,
                ShooterConstants.SHOOTER_RPM_DEADBAND);

        atDesiredPassRPM = 
            MathUtil.isNear( 
                inputs.leftVelocityRPM, inputs.leftTargetVelocityRPM,
                ShooterConstants.SHOOTER_PASS_RPM_DEADBAND)
            && MathUtil.isNear(
                inputs.rightVelocityRPM, inputs.rightTargetVelocityRPM,
                ShooterConstants.SHOOTER_PASS_RPM_DEADBAND);
    }

    /**
     * The function sets both of the motors to a targetVelocity that is
     * the speed provided
     * 
     * @param speed A double representing the speed the motors should be set to
     */
    public void setSpeed(double speed) {
        leftMotor.setTargetVelocity(speed);
        rightMotor.setTargetVelocity(speed);
    }

    public void setSpeed(Pair<Double, Double> speeds) {
        leftMotor.setTargetVelocity(speeds.getFirst());
        rightMotor.setTargetVelocity(speeds.getSecond());
    }

    /**
     * The function is a command that sets the motor speed for both motors
     * to the speed provided
     * 
     * @param speed A double representing the speed the motors should be set to
     * 
     * @return The method is returning a Command object.
     */
    public Command setSpeedCommand(double speed) {
        return Commands.runOnce(() -> setSpeed(speed));
    }

    public Command setSpeedCommand(Pair<Double, Double> speeds) {
        return runOnce(() -> setSpeed(speeds));
    }

    public Pair<Double, Double> getSpeed() {
        return new Pair<Double, Double>(inputs.leftVelocityRPM, inputs.rightVelocityRPM);
    }

    public double getAverageSpeed() {
        return (inputs.leftVelocityRPM + inputs.rightVelocityRPM) / 2.0;
    }

    /**
     * The function is a command that stops both motors
     * 
     * @return The method is returning a Command object.
     */
    public Command stopCommand() {
        return Commands.either(
            setSpeedCommand(0),
            runOnce(() -> {
                leftMotor.set(0);
                rightMotor.set(0);
            }),
            () -> FieldConstants.IS_SIMULATION);
    }
    
    public boolean getAtDesiredRPM() {
        return atDesiredRPM;
    }

    public boolean getAtDesiredPassRPM() {
        return atDesiredPassRPM;
    }

    public Pair<Double, Double> getDesiredSpeeds() {
        return new Pair<Double, Double>(inputs.leftTargetVelocityRPM, inputs.rightTargetVelocityRPM);
    }

    public double getAverageTargetSpeed() {
        return (inputs.leftTargetVelocityRPM + inputs.rightTargetVelocityRPM) / 2.0;
    }

    public Command fullPower(double desiredSpeed) {
        return run(() -> {
            leftMotor.setVoltage(12);
            rightMotor.setVoltage(12);
        }).until(() -> getAverageSpeed() >= desiredSpeed)
        .andThen(setSpeedCommand(desiredSpeed));
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftVelocityRPM = leftMotor.getVelocity();
        inputs.leftTargetVelocityRPM = leftMotor.getTargetVelocity();
        inputs.leftPositionRotations = leftMotor.getPosition();
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput();
        inputs.leftOutputCurrentAmps = leftMotor.getOutputCurrent();

        inputs.rightVelocityRPM = rightMotor.getVelocity();
        inputs.rightTargetVelocityRPM = rightMotor.getTargetVelocity();
        inputs.rightPositionRotations = rightMotor.getPosition();
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput();
        inputs.rightOutputCurrentAmps = rightMotor.getOutputCurrent();
    }
}