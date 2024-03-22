package frc.robot.util.custom.geometry;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * this class is a utility class that contains static methods for transforming
 * Rotation2d objects
 * 
 * Maybe one day this class will be expanded as a general transform that includes 
 * translation and scaling as well as rotation and twisting
 */
public class Transform {
    
    public static boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    public static Rotation2d inverse(Rotation2d rotation2d) {
        return new Rotation2d(rotation2d.getCos(), -rotation2d.getSin());
    }

    public static Rotation2d flip(Rotation2d rotation2d) {
        return new Rotation2d(-rotation2d.getCos(), -rotation2d.getSin());
    }

}
