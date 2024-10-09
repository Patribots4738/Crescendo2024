package frc.robot.util.custom;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * PID constants used to create PID controllers
 * This class is special becuase of its overloaded constructors
 * It just helps keep everything organized
 */
public class PatrIDConstants {

    private final double P; 
    private final double I;
    private final double D;
    private final double FF;
    private final double iZone;
    private final double minOutput;
    private final double maxOutput;

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
        return String.format(
            "P: %.2f | I: %.4f | D: %.2f | FF: %.4f | iZone: %.2f | minOutput: %.1f | maxOutput: %.1f",
            this.P, this.I, this.D, this.FF, this.iZone, this.minOutput, this.maxOutput);
    }
}