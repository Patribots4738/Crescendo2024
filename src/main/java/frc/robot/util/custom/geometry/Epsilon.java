/**
 * This file was inspired by team 254 inside of their Util class
 */

package frc.robot.util.custom.geometry;

public class Epsilon {
    public static final double EPSILON = 1E-12;

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(CustomTwist2d a, CustomTwist2d b, double epsilon) {
        return epsilonEquals(a.dx, b.dx, epsilon) 
            && epsilonEquals(a.dy, b.dy, epsilon) 
            && epsilonEquals(a.dtheta, b.dtheta, epsilon);
    }

}