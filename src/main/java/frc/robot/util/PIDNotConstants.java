package frc.robot.util;

import com.revrobotics.SparkPIDController;

import monologue.Logged;

import com.pathplanner.lib.util.PIDConstants;

public class PIDNotConstants implements Logged {
    /** P */
    public double p;
    /** I */
    public double i;
    /** D */
    public double d;
    /** Integral range */
    public final double iZone;

    private final SparkPIDController PIDController;

    public PIDNotConstants(double p, double i, double d, double iZone, SparkPIDController PIDController) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.iZone = iZone;
        this.PIDController = PIDController;
        this.updatePID();
    }

    /**
     * Create a new PIDConstants object
     *
     * @param p P
     * @param i I
     * @param d D
     */
    public PIDNotConstants(double p, double i, double d, SparkPIDController PIDController) {
        this(p, i, d, 1.0, PIDController);
    }

    /**
     * Create a new PIDConstants object
     *
     * @param p P
     * @param d D
     */
    public PIDNotConstants(double p, double d, SparkPIDController PIDController) {
        this(p, 0, d, PIDController);
    }

    public void updatePID() {
        if (PIDController.getP() != this.p) {
            this.PIDController.setP(this.p);
            System.out.println("Changeing P to: " + p);
        }
        if (PIDController.getI() != this.i) {
            this.PIDController.setI(this.i);
            System.out.println("Changeing I to: " + i);
        }
        if (PIDController.getD() != this.d) {
            this.PIDController.setD(this.d);
            System.out.println("Changeing D to: " + d);
        }
    }

    public String toString() {
        return "P: " + this.p + "\nI: " + this.i + "\nD: " + this.d;
    }

    /**
     * Create a new PIDConstants object
     *
     * @param p P
     */
    public PIDNotConstants(double p, SparkPIDController PIDController) {
        this(p, 0, 0, PIDController);
    }

    public PIDNotConstants(PIDConstants PID, SparkPIDController PIDController) {
        this(PID.kP, PID.kI, PID.kD, 1.0, PIDController);
    }

    public PIDConstants getPID() {
        return new PIDConstants(this.p, this.i, this.d);
    }

    public void setP(double value) {
        p = value;
    }

    public void setI(double value) {
        i = value;
    }

    public void setD(double value) {
        d = value;
    }

    public double getP() {
        return this.p;
    }

    public double getI() {
        return this.i;
    }

    public double getD() {
        return this.d;
    }

}