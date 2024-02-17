package frc.robot.util;

import com.revrobotics.SparkPIDController;

import monologue.Logged;
import monologue.Annotations.Log;

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
    public double kFF;
    /** Integral range */
    public final double iZone = 0;

    private final SparkPIDController[] PIDControllers;

    public PIDNotConstants(double kP, double kI, double kD, double ff, SparkPIDController... PIDControllers) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF= ff;
        this.PIDControllers = PIDControllers;
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public PIDNotConstants(double kP, double kI, double kD, SparkPIDController... PIDControllers) {
        this(kP, kI, kD, 0, PIDControllers);
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

    public PIDNotConstants(PatrIDConstants PID, double ff, SparkPIDController... PIDControllers) {
        this(PID.getP(), PID.getI(), PID.getD(), ff, PIDControllers);
    }

    public PIDNotConstants(PatrIDConstants PID, SparkPIDController... PIDControllers) {
        this(PID.getP(), PID.getI(), PID.getD(), PID.getFF(), PIDControllers);
    }


    public PatrIDConstants getPID() {
        return new PatrIDConstants(this.kP, this.kI, this.kD);
    }

    public void setP(double value) {
        kP = value;
        for (SparkPIDController controller : PIDControllers) {
            controller.setP(value);
        }
    }

    public void setI(double value) {
        kI = value;
        for (SparkPIDController controller : PIDControllers) {
            controller.setI(value);
        }
    }

    public void setD(double value) {
        kD = value;
        for (SparkPIDController controller : PIDControllers) {
            controller.setD(value);
        }
    }

    public void setFF(double value) {
        kFF = value;
        for (SparkPIDController controller : PIDControllers) {
            controller.setFF(value);
        }
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