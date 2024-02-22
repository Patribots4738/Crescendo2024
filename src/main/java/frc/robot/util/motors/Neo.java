// Developed in tandem with Reza from Team Spyder (1622)

package frc.robot.util.motors;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.constants.PatrIDConstants;
import frc.robot.util.constants.Constants.FieldConstants;
import frc.robot.util.constants.Constants.NeoMotorConstants;

/*
 * Some of this is adapted from 3005's 2022 Code
 * Original source published at https://github.com/FRC3005/Rapid-React-2022-Public/tree/d499655448ed592c85f9cfbbd78336d8841f46e2
 */
public class Neo extends SafeSparkBase {

    private ControlLoopType controlType = ControlLoopType.PERCENT;
    private double targetPosition = 0;
    private double targetVelocity = 0;
    private double targetPercent = 0;

    
    /**
     * Creates a new Neo motor with the default settings.
     * @see Neo.NeoBuilder for more configuration options
     * 
     * @param id the CANID of the SparkMax/Flex the Neo is connected to
     */
    public Neo(int id) {
        this(id, false, false);
    }

    /**
     * Creates a new Neo motor with the default settings.
     * @see Neo.NeoBuilder for more configuration options
     * 
     * @param id       the CANID of the SparkMax/Flex the Neo is connected to
     * @param inverted whether the motor is reversed or not
     */
    public Neo(int id, boolean inverted) {
        this(id, inverted, false);
    }

    /**
     * Creates a new Neo motor
     * 
     * @param id       CANID of the SparkMax the Neo is connected to.
     * @param inverted Whether the motor is reversed or not.
     * @param mode     The idle mode of the motor. If true, the motor will brake
     *                 when not powered. If false, the motor will coast when not
     *                 powered.
     */
    public Neo(int id, boolean inverted, boolean useAbsoluteEncoder) {
        super(id, useAbsoluteEncoder, CANSparkLowLevel.MotorType.kBrushless);
        // Set the motor to factory default settings
        // we do this so we can hot-swap sparks without needing to reconfigure them
        // this requires the code to configure the sparks after construction
        restoreFactoryDefaults();
        setInverted(inverted);
        // Add a delay to let the spark reset
        // If a parameter set fails, this will add more time 
        // to minimize any bus traffic.
        // Default is 20ms
        setCANTimeout(50);
        Timer.delay(0.25);
        
        // Turn off alternate and analog encoders
        // we never use them
        setTelemetryPreference(TelemetryPreference.DEFAULT);
        if (useAbsoluteEncoder) {
            setTelemetryPreference(TelemetryPreference.ONLY_ABSOLUTE_ENCODER);
            pidController.setFeedbackDevice(getAbsoluteEncoder());
        } else {
            setTelemetryPreference(TelemetryPreference.ONLY_RELATIVE_ENCODER);
        }
        setBrakeMode();
        register();
    }

    /**
     * Sets the target position for the Neo motor
     * with an optional arbitrary feedforward value.
     * 
     * @param position             The target position for the motor.
     * @param arbitraryFeedForward An optional arbitrary feedforward value.
     */
    public void setTargetPosition(double position, double arbitraryFeedForward) {
        setTargetPosition(position, arbitraryFeedForward, 0);
    }

    /**
     * Sets the target position for the Neo motor
     * with an optional arbitrary feedforward value.
     * 
     * @param position The target position for the motor.
     * @param slot     The PID slot for the reference
     */
    public void setTargetPosition(double position, int slot) {
        setTargetPosition(position, 0, slot);
    }

    /**
     * Sets the target position for the Neo motor.
     * 
     * @param position the desired target position
     */
    public void setTargetPosition(double position) {
        setTargetPosition(position, 0, 0);
    }
    
    /**
     * Sets the target position for the Neo motor controller.
     * 
     * @param position             The desired position in encoder units.
     * @param arbitraryFeedForward The arbitrary feedforward value to be applied.
     * @param slot                 The PID slot to use for control.
     */
    public void setTargetPosition(double position, double arbitraryFeedForward, int slot) {
        if (!FieldConstants.IS_SIMULATION) {
            setPIDReference(position, ControlType.kPosition, slot, arbitraryFeedForward, ArbFFUnits.kVoltage);
        }
        targetPosition = position;
        controlType = ControlLoopType.POSITION;
    }

    /**
     * Sets the target percent for the Neo motor.
     * 
     * @param percent the target percent to set
     */
    public void setTargetPercent(double percent) {
        setTargetPercent(percent, 0);
    }

    /**
     * Sets the target percent of speed output for the Neo.
     * 
     * @param percent Percent of Neo output speed.
     * @param slot The PID slot.
     */
    public void setTargetPercent(double percent, int slot) {
        setTargetPercent(percent, 0, slot);
    }

    /**
     * Sets the target percent of speed output for the Neo.
     * 
     * @param percent              Percent of Neo output speed.
     * @param arbitraryFeedForward The FF for the reference
     */
    public void setTargetPercent(double percent, double arbitraryFeedForward) {
        setTargetPercent(percent, arbitraryFeedForward, 0);
    }

    /**
     * Sets the target percent output for the Neo motor controller.
     * 
     * @param percent              The desired percent output (-1.0 to 1.0).
     * @param arbitraryFeedForward The arbitrary feedforward value.
     * @param slot                 The PID slot to use.
     */
    public void setTargetPercent(double percent, double arbitraryFeedForward, int slot) {
        if (percent == 0) {
            setVoltage(0);
        } else {
            setPIDReference(percent, ControlType.kDutyCycle, slot, arbitraryFeedForward, ArbFFUnits.kPercentOut);
        }
        targetPercent = percent;
        controlType = ControlLoopType.PERCENT;
    }

    /**
     * Sets the target velocity for the Neo.
     * @param velocity Velocity to set the Neo to in rotations per minute.
     */
    public void setTargetVelocity(double velocity) {
        setTargetVelocity(velocity, 0);
    }

    /**
     * Sets the target velocity for the Neo.
     * 
     * @param velocity             Velocity to set the Neo to in rotations per
     *                             minute.
     * @param arbitraryFeedForward The FF for the reference.
     */
    public void setTargetVelocity(double velocity, double arbitraryFeedForward) {
        setTargetVelocity(velocity, arbitraryFeedForward, 0);
    }
    
    /**
     * Sets the target velocity for the Neo motor.
     * 
     * @param velocity the target velocity to set
     * @param slot     the slot to set the target velocity for
     */
    public void setTargetVelocity(double velocity, int slot) {
        setTargetVelocity(velocity, 0, slot);
    }

    /**
     * Sets the target velocity for the Neo.
     * 
     * @param velocity             Velocity to set the Neo to in rotations per
     *                             minute.
     * @param arbitraryFeedForward Arbitrary feed forward to add to the motor
     *                             output.
     */
    public void setTargetVelocity(double velocity, double arbitraryFeedForward, int slot) {
        if (velocity == 0) {
            setVoltage(0);
        } else {
            setPIDReference(velocity, ControlType.kVelocity, slot, arbitraryFeedForward, ArbFFUnits.kVoltage);
        }
        targetVelocity = velocity;
        controlType = ControlLoopType.VELOCITY;
    }

    
    /**
     * Sets the output of the Neo motor controller based on a percentage value.
     * Positive values are counter clockwise if there is no reversedMultiplier
     * 
     * @param percent The percentage value to set the motor output to.
     */
    public void set(double percent) {
        super.set(percent);
        controlType = ControlLoopType.PERCENT;
    }

    /**
     * Updates the position and velocity of the Neo motor.
     * If caching is enabled, it caches the position and velocity values.
     * If the simulation is active and the control type is position, 
     * it simulates movement on the motor
     */
    public void tick() {
        if ((FieldConstants.IS_SIMULATION) && controlType == ControlLoopType.POSITION) {
            setVoltage(getP() * (targetPosition - getPosition()));
        }
    }

    /**
     * Registers the Neo motor instance.
     * Adds the motor to the list of registered motors
     * and also adds the motor to the REVPhysicsSim instance
     * if in simulation.
     */
    public void register() {
        NeoMotorConstants.MOTOR_LIST.add(this);
        if (FieldConstants.IS_SIMULATION)
            REVPhysicsSim.getInstance().addSparkMax(this, DCMotor.getNEO(1));  
    }

    /**
     * Gets the position of the Neo in rotations.
     * @return The position of the Neo in rotations relative to the last 0 position.
     */
    public double getPosition() {
        double pos = super.getPosition();
                
        if (FieldConstants.IS_SIMULATION && controlType == ControlLoopType.VELOCITY) {
            pos /= super.getVelocityConversionFactor();
        }

        return pos;
    }

    /**
     * Gets the velocity of the Neo in rotations per minute.
     * @return The instantaneous velocity of the Neo in rotations per minute.
     */
    public double getVelocity() {
        return super.getVelocity(); 
    }

    /**
     * Sets the position of the Neo encoder.
     * 
     * @param position the desired position to set
     */
    public void resetEncoder(double position) {
        super.setPosition(position);
    }

    /**
     * Gets the target position of the Neo in rotations.
     * @return The target position of the Neo in rotations.
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Gets the target velocity of the Neo in rotations per minute.
     * @return The target velocity of the Neo in rotations per minute.
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Gets the target percentage of the Neo in... well, %
     * @return The target percent speed of the Neo
     */
    public double getTargetPercent() {
        return targetPercent;
    }

    /**
     * Adds a follower to the Neo motor.
     * 
     * @param follower the Neo motor to be added as a follower
     */
    public void addFollower(Neo follower) {
        addFollower(follower, false);
    }

    /**
     * Adds a follower to the Neo motor.
     * 
     * @param follower The Neo motor to be added as a follower.
     * @param invert   Specifies whether the follower should be inverted or not.
     */
    public void addFollower(Neo follower, boolean invert) {
        follower.follow(this, invert);
    }

    /**
     * Sets the PID values and output limits for the Neo motor controller.
     * 
     * @param P          the proportional gain
     * @param I          the integral gain
     * @param D          the derivative gain
     * @param minOutput  the minimum output value
     * @param maxOutput  the maximum output value
     */
    public void setPID(double P, double I, double D, double minOutput, double maxOutput) {
        setPID(new PatrIDConstants(P, I, D, minOutput, maxOutput));
    }

    /**
     * Sets the PID constants for the Neo motor controller.
     * 
     * @param constants the PID constants to set
     */
    public void setPID(PatrIDConstants constants) {
        setPID(constants, 0);
    }

    /**
     * Sets the PID constants for the Neo motor controller.
     * 
     * @param constants the PID constants to set
     * @param slotID    the slot ID of the PID controller
     */
    public void setPID(PatrIDConstants constants, int slot) {
        setPID(
            constants.getP(), 
            constants.getI(), 
            constants.getD(), 
            constants.getFF(),
            constants.getIZone(), 
            constants.getMinOutput(), 
            constants.getMaxOutput(), 
            slot
        );
    }

    /**
     * Sets the PID values and output range for the PID controller.
     * 
     * @param P          the proportional gain
     * @param I          the integral gain
     * @param D          the derivative gain
     * @param minOutput  the minimum output value
     * @param maxOutput  the maximum output value
     * @param slotID     the slot ID of the PID controller
     */
    public void setPID(double P, double I, double D, double FF, double iZone, double minOutput, double maxOutput, int slotID) {
        super.setP(P, slotID);
        super.setI(I, slotID);
        super.setD(D, slotID);
        super.setFF(FF, slotID);
        super.setIZone(iZone, slotID);
        super.setOutputRange(minOutput, maxOutput, slotID);
    }

    /**
     * Gets the proportional gain constant for PID controller.
     * @return The proportional gain constant for PID controller.
     */
    public double getP() {
        return getPIDController().getP();
    }

    /**
     * Gets the integral gain constant for PID controller.
     * @return The integral gain constant for PID controller.
     */
    public double getI() {
        return getPIDController().getI();
    }

    /**
     * Gets the derivative gain constant for PID controller.
     * @return The derivative gain constant for PID controller.
     */
    public double getD() {
        return getPIDController().getD();
    }

    public PatrIDConstants getPID() {
        return new PatrIDConstants(getPIDController().getP(), getPIDController().getI(), getPIDController().getD());
    }

    /**
     * Gets the I-Zone constant for PID controller.
     * The I-Zone is the zone at which the integral
     * will be applied and "enabled"
     *
     * Example: 
     *   an I-Zone of 0.25
     *   will make the integral apply 
     *   for when the absolute value of
     *   | desired - current | is 0.25 or less.
     * 
     * Think of this like the final stretch of PID
     * 
     * @return The I-Zone constant for PID control.
     */
    public double getIZ() {
        return getPIDController().getIZone();
    }

    /**
     * Gets the feedforward gain constant for PID controller.
     * @return The feedforward gain constant for PID controller.
     */
    public double getFF() {
        return getPIDController().getFF();
    }

    /**
     * Enum representing the control loop types for the Neo motor.
     * The control loop types include POSITION, VELOCITY, and PERCENT.
     */
    public enum ControlLoopType {
        POSITION,
        VELOCITY,
        PERCENT;
    }
}