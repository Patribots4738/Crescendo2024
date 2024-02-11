package frc.robot.util;

import com.revrobotics.SparkPIDController;
import com.pathplanner.lib.util.PIDConstants;

public class PIDNotConstants {
    /** P */
    public double kP;
    /** I */
    public double kI;
    /** D */
    public double kD;
    /** Integral range */
    public final double iZone;

    private final SparkPIDController PIDController;

    public PIDNotConstants(double kP, double kI, double kD, double iZone, SparkPIDController PIDController) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
        this.PIDController = PIDController;
        this.updatePID();
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public PIDNotConstants(double kP, double kI, double kD, SparkPIDController PIDController) {
        this(kP, kI, kD, 1.0, PIDController);
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

    public void updatePID() {
        if (PIDController.getP() != this.kP) {
            this.PIDController.setP(this.kP);
        }
        if (PIDController.getI() != this.kI) {
            this.PIDController.setI(this.kI);
        }
        if (PIDController.getD() != this.kD) {
            this.PIDController.setD(this.kD);
        }
    }

    public String toString() {
        return "P: " + this.kP + "\nI: " + this.kI + "\nD: " + this.kD;
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     */
    public PIDNotConstants(double kP, SparkPIDController PIDController) {
        this(kP, 0, 0, PIDController);
    }

    public PIDNotConstants(PIDConstants PID, SparkPIDController PIDController) {
        this(PID.kP, PID.kI, PID.kD, 1.0, PIDController);
    }

    public PIDConstants getPID() {
        return new PIDConstants(this.kP, this.kI, this.kD);
    }

    public void setP(double value) {
        kP = value;
    }

    public void setI(double value) {
        kI = value;
    }

    public void setD(double value) {
        kD = value;
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

}