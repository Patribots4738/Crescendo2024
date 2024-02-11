package frc.robot.util;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import monologue.Logged;
import monologue.Annotations.Log;

import com.pathplanner.lib.util.PIDConstants;

public class PIDNotConstants implements Logged {
    /** P */
    @Log
    public double kP;
    /** I */
    @Log
    public double kI;
    /** D */
    @Log
    public double kD;
    @Log
    public double kFF = 0.001;
    /** Integral range */
    public final double iZone = 0;

    private final SparkPIDController PIDController;

    public PIDNotConstants(double kP, double kI, double kD, double ff, SparkPIDController PIDController) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF= ff;
        this.PIDController = PIDController;
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public PIDNotConstants(double kP, double kI, double kD, SparkPIDController PIDController) {
        this(kP, kI, kD, 0, PIDController);
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     * @param kD D
     */
    public PIDNotConstants(double kP, double kD, SparkPIDController PIDController) {
        this(kP, 0, kD, PIDController);
    }

    public String toString() {
        return String.format("P: %.6f | I: %.6f | D: %.6f | FF: %.6f", this.kP, this.kI, this.kD, this.kFF);
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     */
    public PIDNotConstants(double kP, SparkPIDController PIDController) {
        this(kP, 0, 0, PIDController);
    }

    public PIDNotConstants(PIDConstants PID, double ff, SparkPIDController PIDController) {
        this(PID.kP, PID.kI, PID.kD, ff, PIDController);
    }

    public PIDNotConstants(PIDConstants PID, SparkPIDController PIDController) {
        this(PID.kP, PID.kI, PID.kD, 0, PIDController);
    }

    public PIDConstants getPID() {
        return new PIDConstants(this.kP, this.kI, this.kD);
    }

    public void setP(double value) {
        kP = value;
        this.PIDController.setP(value);
    }

    public void setI(double value) {
        kI = value;
        this.PIDController.setI(value);
    }

    public void setD(double value) {
        kD = value;
        this.PIDController.setD(value);
    }

    public void setFF(double value) {
        kFF = value;
        this.PIDController.setFF(value);
    }

    public double getP() {
        return this.kP;
    }

    public double getI() {
        return this.kI;
    }

    public double getD() {
        return this.kD;
    }

    public double getFF() {
        return this.kFF;
    }

}