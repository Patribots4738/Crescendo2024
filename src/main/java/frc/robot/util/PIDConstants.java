package frc.robot.util;

/** PID constants used to create PID controllers */
public class PIDConstants {
    /** P */
    private final double P;
    /** I */
    private final double I;
    /** D */
    private final double D;
    
    private final double iZone;

    private final double minOutput;
    private final double maxOutput;

    private final double FF;

    public PIDConstants(double kP, double kI, double kD, double FF, double iZone, double minOutput, double maxOutput) {
        this.P = kP;
        this.I = kI;
        this.D = kD;
        this.FF = FF;
        this.iZone = iZone;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public PIDConstants(double P, double I, double D) {
        this(P, I, D, 0, 1.0, -1, 1);
    }

    public PIDConstants(double P, double I, double D, double FF) {
        this(P, I, D, FF, 1.0, -1, 1);
    }

    public PIDConstants(double P, double D) {
        this(P, 0, D, 0, 1.0, -1, 1);
    }

    public PIDConstants(double P, double I, double D, double minOutput, double maxOutput) {
        this(P, I, D, 0, 1.0, minOutput, maxOutput);
    }

    public PIDConstants(double P, double I, double D, double FF, double minOutput, double maxOutput) {
        this(P, I, D, FF, 1.0, minOutput, maxOutput);
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    public double getFF() {
        return FF;
    }

    public double getIZone() {
        return iZone;
    }

    public double getMinOutput() {
        return minOutput;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public String toString() {
        return String.format("P: %.4f | I: %.4f | D: %.4f | FF: %.4f | iZone: %.4f | minOutput: %.4f | maxOutput: %.4f", this.P, this.I, this.D, this.FF, this.iZone, this.minOutput, this.maxOutput);
    }

    /**
     * Returns an array containing the PID constants and feedforward value.
     * The array is in the order [P, I, D, FF].
     *
     * @return an array of doubles representing the PID constants and feedforward value
     */
    public double[] getPIDFF() {
        return new double[] { P, I, D, FF };
    }

    /**
     * Returns an array containing the proportional, integral, and derivative constants for the PID controller.
     *
     * @return an array of doubles representing the PID constants [P, I, D]
     */
    public double[] getPID() {
        return new double[] { P, I, D };
    }

    /**
     * Returns an array containing the PID, FF and iZone constants.
     * 
     * @return an array of doubles containing the PID, FF and iZone constants in the order [P, I, D, FF, iZone]
     */
    public double[] getPIDFFIZone() {
        return new double[] { P, I, D, FF, iZone };
    }

    /**
     * Returns the PID constants in an array
     * 
     * @return An array of the PID constants in the order [P, I, D, FF, iZone, minOutput, maxOutput]
     */
    public double[] getPIDAll() {
        return new double[] { P, I, D, FF, iZone, minOutput, maxOutput };
    }
    
}
