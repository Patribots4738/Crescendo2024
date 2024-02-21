// Primarily referenced from https://github.com/lasarobotics/PurpleLib/
package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.util.Constants.NeoMotorConstants;

public class SafeSpark extends CANSparkMax {

    protected final int canID;
    protected final boolean useAbsoluteEncoder;

    private final int MAX_ATTEMPTS = 20;
    private final int MEASUREMENT_PERIOD = 16;
    private final int AVERAGE_DEPTH = 2;
    private final double BURN_FLASH_WAIT_TIME = 0.5;
    private final double APPLY_PARAMETER_WAIT_TIME = 0.1;

    public SafeSpark(int canID, boolean useAbsoluteEncoder, CANSparkBase.MotorType motorType) {
        super(canID, motorType);

        if (motorType == CANSparkBase.MotorType.kBrushless) {
            fixMeasurementPeriod();
            fixAverageDepth();
        }

        this.canID = canID;
        this.useAbsoluteEncoder = useAbsoluteEncoder;

        if (useAbsoluteEncoder) {
            setTelemetryPreference(TelemetryPreference.ONLY_ABSOLUTE_ENCODER);
            getPIDController().setFeedbackDevice(getAbsoluteEncoder());
        } else {
            setTelemetryPreference(TelemetryPreference.ONLY_RELATIVE_ENCODER);
        }
    }

    /**
     * Attempt to apply parameter and check if specified parameter is set correctly
     * 
     * @param parameterSetter        Method to set desired parameter
     * @param parameterCheckSupplier Method to check for parameter in question
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError applyParameter(Supplier<REVLibError> parameterSetter, BooleanSupplier parameterCheckSupplier,
            String errorMessage) {
        if (RobotBase.isSimulation())
            return parameterSetter.get();
        if (parameterCheckSupplier.getAsBoolean())
            return REVLibError.kOk;

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
     * Check status and print error message if necessary
     * 
     * @param status       Status to check
     * @param errorMessage Error message to print
     */
    public void checkStatus(REVLibError status, String errorMessage) {
        if (status != REVLibError.kOk)
            System.err.println(canID + " (" + NeoMotorConstants.CAN_ID_MAP.get(canID) + ") " + errorMessage + " - "
                    + status.toString());
    }

    /**
     * Set encoder velocity measurement period to {@value Spark#MEASUREMENT_PERIOD}
     * milliseconds
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError fixMeasurementPeriod() {
        REVLibError status;
        status = applyParameter(
                () -> this.getEncoder().setMeasurementPeriod(MEASUREMENT_PERIOD),
                () -> this.getEncoder().getMeasurementPeriod() == MEASUREMENT_PERIOD,
                "Set encoder measurement period failure!");
        return status;
    }

    /**
     * Set encoder velocity average depth to {@value Spark#AVERAGE_DEPTH} samples
     * 
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError fixAverageDepth() {
        REVLibError status;
        status = applyParameter(
                () -> this.getEncoder().setAverageDepth(AVERAGE_DEPTH),
                () -> this.getEncoder().getAverageDepth() == AVERAGE_DEPTH,
                "Set encoder average depth failure!");
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

        Timer.delay(BURN_FLASH_WAIT_TIME);
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
        REVLibError status;
        status = applyParameter(
                () -> super.restoreFactoryDefaults(),
                () -> true,
                "Restore factory defaults failure!");

        return status;
    }

    /**
     * Set a Spark to follow another Spark
     * 
     * @param leader Spark to follow
     * @param invert Set slave to output opposite of the master
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError follow(SafeSpark leader, boolean invert) {
        REVLibError status;
        status = applyParameter(
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
            parameterSetter = () -> getAbsoluteEncoder().setVelocityConversionFactor(factor);
            parameterCheckSupplier = () -> getVelocityConversionFactor() == factor;
        } else {
            parameterSetter = () -> super.getEncoder().setVelocityConversionFactor(factor);
            parameterCheckSupplier = () -> getVelocityConversionFactor() == factor;
        }

        status = applyParameter(parameterSetter, parameterCheckSupplier, "Set position conversion factor failure!");
        return status;
    }

    public double getPositionConversionFactor() {
        if (useAbsoluteEncoder) {
            return getAbsoluteEncoder().getPositionConversionFactor();
        } else {
            return super.getEncoder().getPositionConversionFactor();
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
            parameterSetter = () -> super.getEncoder().setVelocityConversionFactor(factor);
            parameterCheckSupplier = () -> getVelocityConversionFactor() == factor;
        }

        status = applyParameter(parameterSetter, parameterCheckSupplier, "Set velocity conversion factor failure!");
        return status;
    }

    public double getVelocityConversionFactor() {
        if (useAbsoluteEncoder) {
            return getAbsoluteEncoder().getVelocityConversionFactor();
        } else {
            return super.getEncoder().getVelocityConversionFactor();
        }
    }

    public double getPosition() {
        if (useAbsoluteEncoder) {
            return getAbsoluteEncoder().getPosition();
        } else {
            return super.getEncoder().getPosition();
        }
    }

    public void setPosition(double position) {
        super.getEncoder().setPosition(position);
    }

    public double getVelocity() {
        if (useAbsoluteEncoder) {
            return getAbsoluteEncoder().getVelocity();
        } else {
            return super.getEncoder().getVelocity();
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
        REVLibError status;
        status = applyParameter(
                () -> super.getPIDController().setP(value, slot),
                () -> super.getPIDController().getP(slot) == value,
                "Set kP failure!");
        return status;
    }

    /**
     * Set integral gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setI(double value, int slot) {
        REVLibError status;
        status = applyParameter(
                () -> super.getPIDController().setI(value, slot),
                () -> super.getPIDController().getI(slot) == value,
                "Set kI failure!");
        return status;
    }

    /**
     * Set derivative gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setD(double value, int slot) {
        REVLibError status;
        status = applyParameter(
                () -> super.getPIDController().setD(value, slot),
                () -> super.getPIDController().getD(slot) == value,
                "Set kD failure!");
        return status;
    }

    /**
     * Set feed-forward gain for PIDF controller on Spark
     * 
     * @param value Value to set
     * @param slot Slot to set
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setFF(double value, int slot) {
        REVLibError status;
        status = applyParameter(
                () -> super.getPIDController().setFF(value, slot),
                () -> super.getPIDController().getFF(slot) == value,
                "Set kF failure!");
        return status;
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
    public REVLibError setIzone(double value, int slot) {
        REVLibError status;
        status = applyParameter(
                () -> super.getPIDController().setIZone(value, slot),
                () -> super.getPIDController().getIZone(slot) == value,
                "Set Izone failure!");
        return status;
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
        REVLibError status;
        status = applyParameter(
                () -> super.getPIDController().setOutputRange(min, max, slot),
                () -> super.getPIDController().getOutputMin(slot) == min && super.getPIDController().getOutputMax(slot) == max,
                "Set output range failure!");
        return status;
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
        super.getPIDController().setReference(value, controlType, slot, arbitraryFeedForward, arbFFUnits);
    }

    /**
     * Sets the idle mode setting for the Spark
     * 
     * @param mode Idle mode (coast or brake).
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setIdleMode(IdleMode mode) {
        REVLibError status;
        status = applyParameter(
                () -> super.setIdleMode(mode),
                () -> super.getIdleMode() == mode,
                "Set idle mode failure!");
        return status;
    }

    public REVLibError setBrakeMode() {
        return setIdleMode(IdleMode.kBrake);
    }

    public REVLibError setCoastMode() {
        return setIdleMode(IdleMode.kCoast);
    }

    /**
     * Represents an error that can occur while using the REVLib library.
     * Rev docs:
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * 
     * @param frame  The status frame to reset
     * @param period The update period for the status frame.
     * @return error
     */
    public REVLibError changeStatusFrame(StatusFrame frame, int period) {
        REVLibError error = setPeriodicFramePeriod(frame.getFrame(), period);
        // Add a delay to alleviate bus traffic
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
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, minDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, minDelay);
                break;
            // Disable all telemetry that is unrelated to the relative encoder
            case ONLY_RELATIVE_ENCODER:
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_VELO, maxDelay);
                changeStatusFrame(StatusFrame.ABSOLUTE_ENCODER_POS, maxDelay);
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
