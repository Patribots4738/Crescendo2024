// Primarily referenced from https://github.com/lasarobotics/PurpleLib/blob/master/src/main/java/org/lasarobotics/hardware/revrobotics/Spark.java
package lib.rev;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.NeoMotorConstants;

public class SafeSpark extends CANSparkBase {

    protected final boolean isSparkFlex;
    protected final int canID;
    protected final boolean useAbsoluteEncoder;
    SparkPIDController pidController = getPIDController();
    protected RelativeEncoder relativeEncoder;
    protected SparkAbsoluteEncoder absoluteEncoder;

    private final int MAX_ATTEMPTS = (NeoMotorConstants.SAFE_SPARK_MODE) ? 20 : 2;
    private final int SPARK_MAX_MEASUREMENT_PERIOD = 16;
    private final int SPARK_FLEX_MEASUREMENT_PERIOD = 32;
    private final int SPARK_MAX_AVERAGE_DEPTH = 2;
    private final int SPARK_FLEX_AVERAGE_DEPTH = 8;
    private final double BURN_FLASH_WAIT_TIME = (NeoMotorConstants.SAFE_SPARK_MODE) ? 0.1 : 0.05;
    private final double APPLY_PARAMETER_WAIT_TIME = (NeoMotorConstants.SAFE_SPARK_MODE) ? 0.05 : 0;

    public SafeSpark(int canID, boolean useAbsoluteEncoder, MotorType motorType, boolean isSparkFlex) {
        super(canID, motorType, isSparkFlex ? SparkModel.SparkFlex : SparkModel.SparkMax);

        this.canID = canID;
        this.useAbsoluteEncoder = useAbsoluteEncoder;
        this.isSparkFlex = isSparkFlex;

        // Set the motor to factory default settings
        // we do this so we can hot-swap sparks without needing to reconfigure them
        // this requires the code to configure the sparks after construction
        restoreFactoryDefaults();

        if (useAbsoluteEncoder) {
            setFeedbackDevice(getAbsoluteEncoder());
        }
        if (motorType == CANSparkBase.MotorType.kBrushless) {
            fixMeasurementPeriod();
            fixAverageDepth();
        }
    }

    public int getCANID() {
        return canID;
    }

    /**
     * Attempt to apply parameter and check if specified parameter is set correctly
     * 
     * @param parameterSetter        Method to set desired parameter
     * @param parameterCheckSupplier Method to check for parameter in question
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError applyParameter(
        Supplier<REVLibError> parameterSetter, 
        BooleanSupplier parameterCheckSupplier, 
        String errorMessage)
    {
        if (FieldConstants.IS_SIMULATION)
            return parameterSetter.get();

        REVLibError status = REVLibError.kError;
        for (int i = 0; i < MAX_ATTEMPTS; i++) {
            status = parameterSetter.get();
            if (parameterCheckSupplier.getAsBoolean() && status == REVLibError.kOk)
                break;
            Timer.delay(APPLY_PARAMETER_WAIT_TIME);
        }

        checkStatus(status, errorMessage);
        return status;
    }

    /**
     * Writes all settings to flash
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    @Override
    public REVLibError burnFlash() {
        if (RobotBase.isSimulation())
            return REVLibError.kOk;

            REVLibError status = super.burnFlash();
            Timer.delay(BURN_FLASH_WAIT_TIME);

        return status;
    }

    /**
     * Restore motor controller parameters to factory defaults until the next
     * controller reboot
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    @Override
    public REVLibError restoreFactoryDefaults() {
        REVLibError status = applyParameter(
            () -> super.restoreFactoryDefaults(),
            () -> true,
            "Restore factory defaults failure!");
        Timer.delay(BURN_FLASH_WAIT_TIME);
        return status;
    }

    /**
     * Check status and print error message if necessary
     * 
     * @param status       Status to check
     * @param errorMessage Error message to print
     */
    public void checkStatus(REVLibError status, String errorMessage) {
        if (status != REVLibError.kOk) {
            System.err.println(canID + " (" + NeoMotorConstants.CAN_ID_MAP.get(canID) + ") " + errorMessage + " - "
                    + status.toString());
        }
        if (getFault(FaultID.kSensorFault)) {
            String message = "\nSensor fault detected on motor " +
                canID + " (" + NeoMotorConstants.CAN_ID_MAP.get(canID) + ")" +
                ". Power cycle the robot to fix.";
            for (int i = 0; i < 5; i++) {
                System.err.println(message);
                Timer.delay(.1);
            }
        }
    }

    @Override
    public RelativeEncoder getEncoder() {
        if (relativeEncoder == null) {
            if (isSparkFlex) {
                relativeEncoder = super.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
            } else {
                relativeEncoder = super.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
            }
        }
        return relativeEncoder;
    }

    public RelativeEncoder getRelativeEncoder() {
        return this.getEncoder();
    }

    @Override
    public SparkAbsoluteEncoder getAbsoluteEncoder() {
        if (absoluteEncoder == null) {
            absoluteEncoder = super.getAbsoluteEncoder();
        }
        return absoluteEncoder;
    }

    /**
     * Set encoder velocity measurement period to {@value Spark#MEASUREMENT_PERIOD}
     * milliseconds
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError fixMeasurementPeriod() {
        final int MEASUREMENT_PERIOD = isSparkFlex ? SPARK_FLEX_MEASUREMENT_PERIOD : SPARK_MAX_MEASUREMENT_PERIOD;
        REVLibError status = applyParameter(
                () -> getEncoder().setMeasurementPeriod(MEASUREMENT_PERIOD),
                () -> getEncoder().getMeasurementPeriod() == MEASUREMENT_PERIOD,
                "Set encoder measurement period failure!");
        return status;
    }

    /**
     * Set encoder velocity average depth to {@value Spark#AVERAGE_DEPTH} samples
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError fixAverageDepth() {
        final int AVERAGE_DEPTH = isSparkFlex ? SPARK_FLEX_AVERAGE_DEPTH : SPARK_MAX_AVERAGE_DEPTH;
        REVLibError status = applyParameter(
                () -> getEncoder().setAverageDepth(AVERAGE_DEPTH),
                () -> getEncoder().getAverageDepth() == AVERAGE_DEPTH,
                "Set encoder average depth failure!");
        return status;
    }

    /**
     * Invert the motor.
     * 
     * @param inverted to invert or not to invert, that is the question
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError invertMotor(boolean inverted) {
        return applyParameter(
                () -> {
                    throwIfClosed();
                    if (useAbsoluteEncoder) {
                        return getAbsoluteEncoder().setInverted(inverted);
                    } else {    
                        return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetInverted(sparkMaxHandle, inverted));
                    }
                },
                () -> super.getInverted() == inverted,
                "Set inverted failure!");
    }

    /**
     * Invert the motor
     * 
     */
    @Override
    public void setInverted(boolean inverted) {
        invertMotor(inverted);
    }
    
    /**
     * Set a Spark to follow another Spark
     * 
     * @param leader Spark to follow
     * @param invert Set slave to output opposite of the master
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError follow(SafeSpark leader, boolean invert) {
        REVLibError status = applyParameter(
                () -> super.follow(ExternalFollower.kFollowerSpark, leader.canID, invert),
                () -> super.isFollower(),
                "Set motor master failure!");
        return status;
    }

    /**
     * Set the conversion factor for position of the encoder. Multiplied by the
     * native output units to
     * give you position.
     * 
     * @param sensor Sensor to set conversion factor for
     * @param factor The conversion factor to multiply the native units by
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setPositionConversionFactor(double factor) {
        REVLibError status;
        Supplier<REVLibError> parameterSetter;
        BooleanSupplier parameterCheckSupplier;

        if (useAbsoluteEncoder) {
            parameterSetter = () -> getAbsoluteEncoder().setPositionConversionFactor(factor);
            parameterCheckSupplier = () -> getPositionConversionFactor() == factor;
        } else {
            parameterSetter = () -> getEncoder().setPositionConversionFactor(factor);
            parameterCheckSupplier = () -> getPositionConversionFactor() == factor;
        }

        status = applyParameter(parameterSetter, parameterCheckSupplier, "Set position conversion factor failure!");
        
        return status;
    }

    public double getPositionConversionFactor() {
        if (useAbsoluteEncoder) {
            return getAbsoluteEncoder().getPositionConversionFactor();
        } else {
            return getRelativeEncoder().getPositionConversionFactor();
        }
    }

    /**
     * Set the conversion factor for velocity of the encoder. Multiplied by the
     * native output units to
     * give you velocity.
     * 
     * @param sensor Sensor to set conversion factor for
     * @param factor The conversion factor to multiply the native units by
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setVelocityConversionFactor(double factor) {
        REVLibError status;
        Supplier<REVLibError> parameterSetter;
        BooleanSupplier parameterCheckSupplier;

        if (useAbsoluteEncoder) {
            parameterSetter = () -> getAbsoluteEncoder().setVelocityConversionFactor(factor);
            parameterCheckSupplier = () -> getVelocityConversionFactor() == factor;
        } else {
            parameterSetter = () -> getRelativeEncoder().setVelocityConversionFactor(factor);
            parameterCheckSupplier = () -> getVelocityConversionFactor() == factor;
        }

        status = applyParameter(
            parameterSetter, 
            parameterCheckSupplier, 
            "Set velocity conversion factor failure!");
        
        return status;
    }

    public REVLibError setSoftLimit(double min, double max) {
        REVLibError status;
        Supplier<REVLibError> parameterSetter = () -> {
            REVLibError forwardLimitStatus = super.setSoftLimit(SoftLimitDirection.kForward, (float)min);
            REVLibError reverseLimitStatus = super.setSoftLimit(SoftLimitDirection.kReverse, (float)max);
            REVLibError softLimitStatus = setSoftLimit(true);
        
            if (forwardLimitStatus != REVLibError.kOk 
                || reverseLimitStatus != REVLibError.kOk 
                || softLimitStatus != REVLibError.kOk) 
            {
                return REVLibError.kInvalid;
            }
        
            return REVLibError.kOk;
        };
        BooleanSupplier parameterCheckSupplier = () -> super.getSoftLimit(SoftLimitDirection.kForward) == min &&
                super.getSoftLimit(SoftLimitDirection.kReverse) == max;

        status = applyParameter(parameterSetter, parameterCheckSupplier, "Set soft limits failure!");
        return status;
    }

    public REVLibError setSoftLimit(boolean enable) {
        REVLibError statusForward = applyParameter(
                () -> super.enableSoftLimit(SoftLimitDirection.kForward, enable),
                () -> super.isSoftLimitEnabled(SoftLimitDirection.kForward) == enable,
                enable ? "Enable" : "Disable" + " soft limit forward failure!");

        REVLibError statusReverse = applyParameter(
                () -> super.enableSoftLimit(SoftLimitDirection.kReverse, enable),
                () -> super.isSoftLimitEnabled(SoftLimitDirection.kReverse) == enable,
                enable ? "Enable" : "Disable" + " soft limit reverse failure!");

        return statusForward == REVLibError.kOk && statusReverse == REVLibError.kOk ? REVLibError.kOk : REVLibError.kError;
    }

    public double getVelocityConversionFactor() {
        if (useAbsoluteEncoder) {
            return getAbsoluteEncoder().getVelocityConversionFactor();
        } else {
            return getRelativeEncoder().getVelocityConversionFactor();
        }
    }

    /**
     * Get the position of the encoder
     * This will go through the positionConversionFactor if there is one
     * @return The position of the encoder
     */
    public double getPosition() {
        if (useAbsoluteEncoder && !FieldConstants.IS_SIMULATION) {
            return getAbsoluteEncoder().getPosition();
        } else {
            return getRelativeEncoder().getPosition();
        }
    }

    /**
     * RESET THE RELATIVE ENCODER TO BE THE SET POSITION
     */
    public void setPosition(double position) {
        getRelativeEncoder().setPosition(position);
    }

    /**
     * Get the velocity of the encoder, in RPM
     * This will go through the velocityConversionFactor if there is one
     * @return The velocity of the encoder, in RPM
     */
    public double getVelocity() {
        if (useAbsoluteEncoder) {
            return getAbsoluteEncoder().getVelocity();
        } else {
            return getRelativeEncoder().getVelocity();
        }
    }

    /**
     * Set proportional gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setP(double value, int slot) {
        REVLibError status = applyParameter(
            () -> pidController.setP(value, slot),
            () -> pidController.getP(slot) == value,
            "Set kP failure!");
        return status;
    }

    public REVLibError setP(double value) {
        return setP(value, 0);
    }

    /**
     * Set integral gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setI(double value, int slot) {
        REVLibError status = applyParameter(
            () -> pidController.setI(value, slot),
            () -> pidController.getI(slot) == value,
            "Set kI failure!");
        return status;
    }

    public REVLibError setI(double value) {
        return setI(value, 0);
    }

    /**
     * Set derivative gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setD(double value, int slot) {
        REVLibError status = applyParameter(
            () -> pidController.setD(value, slot),
            () -> pidController.getD(slot) == value,
            "Set kD failure!");
        return status;
    }

    public REVLibError setD(double value) {
        return setD(value, 0);
    }

    /**
     * Set feed-forward gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setFF(double value, int slot) {
        REVLibError status = applyParameter(
            () -> pidController.setFF(value, slot),
            () -> pidController.getFF(slot) == value,
            "Set kF failure!");
        return status;
    }

    public REVLibError setFF(double value) {
        return setFF(value, 0);
    }

    /**
     * Set integral zone range for PIDF controller on Spark
     * <p>
     * This value specifies the range the |error| must be within for the integral
     * constant to take effect
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setIZone(double value, int slot) {
        REVLibError status = applyParameter(
            () -> pidController.setIZone(value, slot),
            () -> pidController.getIZone(slot) == value,
            "Set Izone failure!");
        return status;
    }

    public REVLibError setIZone(double value) {
        return setIZone(value, 0);
    }
    
    /**
     * Set the min/maximum output for the PIDF controller on Spark
     * 
     * @param min Minimum output value
     * @param max Maximum output value
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setOutputRange(double min, double max, int slot) {
        REVLibError status = applyParameter(
            () -> pidController.setOutputRange(min, max, slot),
            () -> pidController.getOutputMin(slot) == min && pidController.getOutputMax(slot) == max,
            "Set output range failure!");
        return status;
    }

    /**
     * Enable PID wrapping for closed loop position control
     * 
     * @param minInput Value of min input for position
     * @param maxInput Value of max input for position
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError enablePIDWrapping(double minInput, double maxInput) {
        Supplier<REVLibError> parameterSetter = () -> {
            REVLibError wrappingEnabledStatus = pidController.setPositionPIDWrappingEnabled(true);
            REVLibError minInputStatus = pidController.setPositionPIDWrappingMinInput(minInput);
            REVLibError maxInputStatus = pidController.setPositionPIDWrappingMaxInput(maxInput);

            if (wrappingEnabledStatus != REVLibError.kOk 
                || minInputStatus != REVLibError.kOk 
                || maxInputStatus != REVLibError.kOk) 
            {
                return REVLibError.kInvalid;
            }

            return REVLibError.kOk;
        };
        BooleanSupplier parameterCheckSupplier = () -> pidController
                .getPositionPIDWrappingEnabled() == true &&
                pidController.getPositionPIDWrappingMinInput() == minInput &&
                pidController.getPositionPIDWrappingMaxInput() == maxInput;

        return applyParameter(
            parameterSetter, 
            parameterCheckSupplier, 
            "Enable position PID wrapping failure!");
    }

    /**
     * Disable PID wrapping for close loop position control
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError disablePIDWrapping() {
        return applyParameter(
            () -> pidController.setPositionPIDWrappingEnabled(false),
            () -> pidController.getPositionPIDWrappingEnabled() == false,
            "Disable position PID wrapping failure!");
    }

    /**
     * Set the motor fedeback device to the PIDController
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setFeedbackDevice(MotorFeedbackSensor device) {
        return applyParameter(
            () -> pidController.setFeedbackDevice(device), 
            () -> true,
            "Feedback device failure!");
    }

    /**
     * Set the PID wrapping to be enabled or disabled
     * 
     * @param enabled Whether to enable or disable the PID wrapping
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setPositionPIDWrappingEnabled(boolean enabled) {
        return applyParameter(
            () -> pidController.setPositionPIDWrappingEnabled(enabled),
            () -> pidController.getPositionPIDWrappingEnabled() == enabled,
            "Set PID wrapping enabled failure!");
    }

    public REVLibError setPositionPIDWrappingBounds(double min, double max) {
        return applyParameter(
            () -> {
                REVLibError minStatus = pidController.setPositionPIDWrappingMinInput(min);
                REVLibError maxStatus = pidController.setPositionPIDWrappingMaxInput(max);
                return minStatus == REVLibError.kOk && maxStatus == REVLibError.kOk ? REVLibError.kOk : REVLibError.kInvalid;
            },
            () -> pidController.getPositionPIDWrappingMinInput() == min && pidController.getPositionPIDWrappingMaxInput() == max,
            "Set PID wrapping bounds failure!");
    }

    /**
     * Set the PIDF controller reference for the Spark
     * 
     * @param value        The value to set the reference to
     * @param controlType  The control type to use
     * @param slot         The slot to set
     * @param arbFF        Arbitrary feed forward value
     * @param arbFFUnits   Units for the arbitrary feed forward value
     */
    public void setPIDReference(double value, ControlType controlType, int slot, double arbitraryFeedForward, ArbFFUnits arbFFUnits) {
        pidController.setReference(value, controlType, slot, arbitraryFeedForward, arbFFUnits);
    }

    /**
     * Sets the idle mode setting for the Spark
     * 
     * @param mode Idle mode (coast or brake).
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setIdleMode(IdleMode mode) {
        REVLibError status = applyParameter(
            () -> super.setIdleMode(mode),
            () -> super.getIdleMode() == mode,
            "Set idle mode failure!");
        return status;
    }

    /**
     * Sets the brake mode for the Spark MAX motor controller.
     * 
     * @return The REVLibError indicating the success or failure of the operation.
     */
    public REVLibError setBrakeMode() {
        return this.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Sets the motor controller to coast mode.
     * 
     * @return The REVLibError indicating the success or failure of the operation.
     */
    public REVLibError setCoastMode() {
        return this.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Sets the current limit in Amps.
     *
     * <p>
     * The motor controller will reduce the controller voltage output to avoid
     * surpassing this
     * limit. This limit is enabled by default and used for brushless only. This
     * limit is highly
     * recommended when using the NEO brushless motor.
     *
     * <p>
     * The NEO Brushless Motor has a low internal resistance, which can mean large
     * current spikes
     * that could be enough to cause damage to the motor and controller. This
     * current limit provides a
     * smarter strategy to deal with high current draws and keep the motor and
     * controller operating in
     * a safe region.
     *
     * @param limit The current limit in Amps.
     */
    public REVLibError setSmartCurrentLimit(int limit) {
        return applyParameter(
            () -> super.setSmartCurrentLimit(limit),
            () -> true,
            "Set current limit failure!");
    }

    /**
     * Set the free speed of the motor being simulated.
     *
     * @param freeSpeed the free speed (RPM) of the motor connected to SPARK
     * @return {@link REVLibError#kOk} if successful
     */
    @SuppressWarnings("all")
    public REVLibError setSimFreeSpeed(float freeSpeed) {
        return applyParameter(
                () -> {
                    throwIfClosed();
                    return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSimFreeSpeed(sparkMaxHandle, freeSpeed));
                },
                () -> true,
                "Set Sim Free Speed failure!");
    }

    /**
     * Set the stall torque of the motor being simulated.
     *
     * @param stallTorque The stall torque (N m) of the motor connected to SPARK
     * @return {@link REVLibError#kOk} if successful
     */
    @SuppressWarnings("all")
    public REVLibError setSimStallTorque(float stallTorque) {
        return applyParameter(
                () -> {
                    throwIfClosed();
                    return REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSimStallTorque(sparkMaxHandle, stallTorque));
                },
                () -> true,
                "Set Sim Stall Torque failure!");
    }

    /**
     * Change a periodic status frame period of the motor controller.
     * Rev docs:
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * 
     * You can increase this number to ignore updates more / alliviate can bus traffic
     * or decrease this number to get more frequent updates
     * 
     * @param frame  The frame to change
     * @param period The period to set in milliseconds
     * @return A REVLibError indicating the result of the operation
     */
    public REVLibError changeStatusFrame(StatusFrame frame, int period) {
        REVLibError error = setPeriodicFramePeriod(frame.getFrame(), period);
        // Add a delay to alleviate bus traffic
        if (!FieldConstants.IS_SIMULATION)
            Timer.delay(0.05);

        return error;
    }

    /**
     * Resets the status frame of the NEO motor controller to its default period.
     * Rev docs:
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * 
     * @param frame the status frame to reset
     * @return the REVLibError indicating the result of the operation
     */
    public REVLibError resetStatusFrame(StatusFrame frame) {
        return changeStatusFrame(frame, frame.getDefaultPeriodms());
    }

    /**
     * Represents the status frames for the Neo class.
     * Rev docs:
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     */
    public enum StatusFrame {
        APPLIED_FAULTS_FOLLOWER(PeriodicFrame.kStatus0, 10),
        VELO_TEMP_VOLTAGE_CURRENT(PeriodicFrame.kStatus1, 20),
        ENCODER_POSITION(PeriodicFrame.kStatus2, 20),
        ALL_ANALOG_ENCODER(PeriodicFrame.kStatus3, 50),
        ALL_ALTERNATE_ENCODER(PeriodicFrame.kStatus4, 20),
        ABSOLUTE_ENCODER_POS(PeriodicFrame.kStatus5, 200),
        ABSOLUTE_ENCODER_VELO(PeriodicFrame.kStatus6, 200);

        private final PeriodicFrame frame;
        private final int defaultPeriodms;

        /**
         * Constructs a StatusFrame with the specified frame and default period.
         * 
         * @param frame         The periodic frame.
         * @param defaultPeriod The default period in milliseconds.
         */
        StatusFrame(PeriodicFrame frame, int defaultPeriod) {
            this.frame = frame;
            this.defaultPeriodms = defaultPeriod;
        }

        /**
         * Gets the periodic frame associated with this StatusFrame.
         * 
         * @return The periodic frame.
         */
        public PeriodicFrame getFrame() {
            return frame;
        }

        /**
         * Gets the default period in milliseconds for this StatusFrame.
         * 
         * @return The default period in milliseconds.
         */
        public int getDefaultPeriodms() {
            return defaultPeriodms;
        }
    }

    public enum TelemetryPreference {
        DEFAULT,
        ONLY_ABSOLUTE_ENCODER,
        ONLY_RELATIVE_ENCODER,
        NO_TELEMETRY,
        NO_ENCODER
    }

    /**
     * Set the telemetry preference of the Neo
     * This will disable the telemtry status frames
     * which is found at
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * 
     * @param type the enum to represent the telemetry preference
     *             this will tell the motor to only send
     *             that type of telemtry
     */
    public void setTelemetryPreference(TelemetryPreference type) {
        int minDelay = NeoMotorConstants.FAST_PERIODIC_STATUS_TIME_MS;
        int maxDelay = NeoMotorConstants.MAX_PERIODIC_STATUS_TIME_MS;

        // No matter what preference, we don't use analog or external encoders.
        changeStatusFrame(StatusFrame.ALL_ALTERNATE_ENCODER, maxDelay);
        changeStatusFrame(StatusFrame.ALL_ANALOG_ENCODER, maxDelay);

        switch (type) {
            // Disable all telemetry that is unrelated to the encoder
            case NO_ENCODER:
                changeStatusFrame(StatusFrame.ENCODER_POSITION, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, maxDelay);
                break;
            // Disable all telemetry that is unrelated to absolute encoders
            case ONLY_ABSOLUTE_ENCODER:
                changeStatusFrame(StatusFrame.ENCODER_POSITION, maxDelay);
                changeStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, minDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, minDelay);
                break;
            // Disable all telemetry that is unrelated to the relative encoder
            case ONLY_RELATIVE_ENCODER:
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, maxDelay);
                changeStatusFrame(StatusFrame.ENCODER_POSITION, minDelay);
                changeStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT, minDelay);
                break;
            // Disable everything
            case NO_TELEMETRY:
                changeStatusFrame(StatusFrame.VELO_TEMP_VOLTAGE_CURRENT, maxDelay);
                changeStatusFrame(StatusFrame.ENCODER_POSITION, maxDelay);
                changeStatusFrame(StatusFrame.ALL_ANALOG_ENCODER, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, maxDelay);
                break;

            case DEFAULT:
            default:
                break;
        }
        
    }

}
