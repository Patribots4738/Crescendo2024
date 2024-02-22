package frc.robot.util;

import monologue.Logged;
import monologue.Annotations.Log;


/** PID constants used to create PID controllers */
public class PatrIDConstants implements Logged {

    @Log
    private final double 
        P, 
        I, 
        D, 
        FF, 
        iZone,
        minOutput,
        maxOutput;

    public PatrIDConstants(double P) {
        this(P, 0);
    }

    public PatrIDConstants(double P, double D) {
        this(P, 0, D);
    }

    public PatrIDConstants(double P, double I, double D) {
        this(P, I, D, 0);
    }

    public PatrIDConstants(double P, double I, double D, double FF) {
        this(P, I, D, FF, -1, 1);
    }
    
    public PatrIDConstants(double P, double I, double D, double minOutput, double maxOutput) {
        this(P, I, D, 0, minOutput, maxOutput);
    }

    public PatrIDConstants(double P, double I, double D, double FF, double minOutput, double maxOutput) {
        this(P, I, D, FF, Double.POSITIVE_INFINITY, minOutput, maxOutput);
    }

    public PatrIDConstants(double P, double I, double D, double FF, double iZone, double minOutput, double maxOutput) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.FF = FF;
        this.iZone = iZone;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
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
        return String.format("P: %.4f | I: %.4f | D: %.4f | FF: %.4f | iZone: %.4f | minOutput: %.4f | maxOutput: %.4f",
                this.P, this.I, this.D, this.FF, this.iZone, this.minOutput, this.maxOutput);
    }
}