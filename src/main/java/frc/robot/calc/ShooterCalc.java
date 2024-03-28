package frc.robot.calc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.NTConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.logging.NT;
import lib.SpeedAngleTriplet;

public class ShooterCalc {

    private ShooterCalc() {
    }

    /**
     * Calculates the shooter speeds required to reach the speaker position.
     * 
     * @param pose   a supplier of the robot's current pose
     * @param speeds a supplier of the robot's current chassis speeds
     * @param dt     the time interval for integration
     * @return a pair of shooter speeds (left and right) required to reach the
     *         speaker position
     */
    public static SpeedAngleTriplet calculateSWDTriplet(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d currentPose = pose;

        SpeedAngleTriplet currentTriplet = calculateSpeakerTriplet(pose.getTranslation());
        double normalVelocity = PoseCalculations.getVelocityVectorToSpeaker(currentPose, speeds).getY();

        double originalv0 = rpmToVelocity(currentTriplet.getSpeeds());
        double v0z = originalv0 * Math.sin(Units.degreesToRadians(currentTriplet.getAngle()));
        double v0x = originalv0 * Math.cos(Units.degreesToRadians(currentTriplet.getAngle())) + normalVelocity;

        double newv0 = Math.hypot(v0x, v0z);
        Rotation2d newAngle = new Rotation2d(v0x, v0z);

        return SpeedAngleTriplet.of(
                currentTriplet.getLeftSpeed(),
                currentTriplet.getRightSpeed(),
                newAngle.getDegrees());
    }

    public static SpeedAngleTriplet calculateSWDTripletAuto(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d currentPose = pose;

        SpeedAngleTriplet currentTriplet = calculateSWDTriplet(currentPose, speeds);

        double maxRPMAuto = ShooterConstants.INTERPOLATION_MAP.get(Units.metersToFeet(5)).getAverageSpeed();

        return SpeedAngleTriplet.of(
                MathUtil.clamp(currentTriplet.getLeftSpeed(), 0, maxRPMAuto),
                MathUtil.clamp(currentTriplet.getRightSpeed(), 0, maxRPMAuto),
                currentTriplet.getAngle());
    }
    
    public static SpeedAngleTriplet calculatePassTriplet(Pose2d robotPose, ChassisSpeeds speeds) {
        Rotation2d pivotAngle = calculatePassPivotAngle(robotPose);
        Pair<Number, Number> shooterSpeeds = calculateShooterSpeedsForPassApex(robotPose, pivotAngle);
        return SpeedAngleTriplet.of(
                // Don't ask. It works. Is this how we finally beat the hawaiian kids?
                shooterSpeeds.getFirst(),
                shooterSpeeds.getSecond(),
                pivotAngle.getDegrees());
    }
    
    
    /**
     * Calculates the shooter speeds required to reach the speaker position.
     * 
     * @param pose   a supplier of the robot's current pose
     * 
     * @return a pair of shooter speeds (left and right) required to reach the
     *         speaker position
     */
    public static SpeedAngleTriplet calculateApexTriplet(Pose2d robotPose) {
        Rotation2d pivotAngle = calculatePivotAngle(robotPose);
        Pair<Number, Number> shooterSpeeds = calculateShooterSpeedsForSpeakerApex(robotPose, pivotAngle);
        return SpeedAngleTriplet.of(
                // Not Working
                // Don't ask. It works. Is this how we finally beat the hawaiian kids?
                shooterSpeeds.getFirst(),
                shooterSpeeds.getSecond(),
                pivotAngle.getDegrees());
    }

    /**
     * Calculates the pivot angle based on the robot's pose.
     * 
     * @param robotPose The pose of the robot.
     * @return The calculated pivot angle.
     */
    public static Rotation2d calculatePivotAngle(Pose2d robotPose) {
        // Calculate the robot's pose relative to the speaker's position
        robotPose = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());

        // Calculate the distance in meters from the robot to the speaker
        double distanceMeters = robotPose.getTranslation().getNorm();

        // Return a new rotation object that represents the pivot angle
        // The pivot angle is calculated based on the speaker's height and the distance
        // to the speaker
        return new Rotation2d(
                distanceMeters - NTConstants.PIVOT_OFFSET_METERS.getX(),
                FieldConstants.SPEAKER_HEIGHT_METERS - NTConstants.PIVOT_OFFSET_METERS.getZ() + NT.getValue("atan++"));
    }

    /**
     * Calculates the pivot angle based on the robot's pose.
     * 
     * @param robotPose The pose of the robot.
     * @return The calculated pivot angle.
     */
    public static Rotation2d calculatePassPivotAngle(Pose2d robotPose) {
        // Calculate the robot's pose relative to the speaker's position
        robotPose = robotPose.relativeTo(FieldConstants.GET_PASS_APEX_POSITION());

        // Calculate the distance in feet from the robot to the speaker
        double distanceMeters = robotPose.getTranslation().getNorm();

        // Return a new rotation object that represents the pivot angle
        // The pivot angle is calculated based on the speaker's height and the distance
        // to the speaker
        return new Rotation2d(distanceMeters, FieldConstants.PASS_HEIGHT_METERS + NT.getValue("atan++"));
    }

    /**
     * Calculates the speed and angle triplet for the shooter based on the robot's pose.
     *
     * @param robotPose the translation2d object representing the robot's pose
     * @return the speed and angle triplet for the shooter
     */
    public static SpeedAngleTriplet calculateSpeakerTriplet(Translation2d robotPose) {
        // Get our position relative to the desired field element
        // Use the distance as our key for interpolation
        double distanceFeet = Units.metersToFeet(robotPose.getDistance(FieldConstants.GET_SPEAKER_TRANSLATION()));

        return ShooterConstants.INTERPOLATION_MAP.get(distanceFeet);
    }

    /**
     * This method is averaging the speeds to make a rough estimate of the speed of
     * the note (or the edge of the wheels).
     * The formula used is V = 2π * D/2 * RPM/60.
     * First, it converts from Rotations per Minute to Rotations per Second.
     * Then, it converts from Rotations per Second to Radians per Second.
     * Finally, it multiplies by the radius of the wheel contacting it.
     * As v = rw (w = omega | angular velocity of wheel).
     * 
     * Converts RPM (Revolutions Per Minute) to velocity in meters per second.
     * 
     * @param speeds the average RPM value representing the speed of the shooter
     * @return the velocity in meters per second
     */
    public static double rpmToVelocity(double averageShooterSpeed) {
        double rotationsPerMinute = averageShooterSpeed;
        double rotationsPerSecond = rotationsPerMinute / 60.0;
        double radiansPerSecond = rotationsPerSecond * Math.PI;

        double diameter = ShooterConstants.WHEEL_DIAMETER_METERS;

        // Normally this is 1 radius * 2 pi
        // but we are doing 2 radius * 1 pi
        // because we are given a diameter
        return diameter * radiansPerSecond;
    }

    /**
     * This method is averaging the speeds to make a rough estimate of the speed of
     * the note (or the edge of the wheels).
     * The formula used is V = 2π * D/2 * RPM/60.
     * First, it converts from Rotations per Minute to Rotations per Second.
     * Then, it converts from Rotations per Second to Radians per Second.
     * Finally, it multiplies by the radius of the wheel contacting it.
     * As v = rw (w = omega | angular velocity of wheel).
     * 
     * Converts RPM (Revolutions Per Minute) to velocity in meters per second.
     * 
     * @param speeds a pair of RPM values representing the speeds of two shooter
     *               wheels
     * @return the velocity in meters per second
     */
    public static double rpmToVelocity(Pair<Double, Double> speeds) {
        return rpmToVelocity((speeds.getFirst() + speeds.getSecond()) / 2.0);
    }

    /**
     * Converts the velocity of the note to RPM (Rotations Per Minute).
     * Equation: ((V/(2π)) / (D/2)) * 60 = RPM
     * 
     * @param noteVelocity the velocity of the initial note in meters per second
     * @return the RPM (Rotations Per Minute) of the shooter wheel
     */
    public static double velocityToRPM(double noteVelocity) {
        double diameter = ShooterConstants.WHEEL_DIAMETER_METERS;

        // Convert velocity back to radians per second
        double radiansPerSecond = noteVelocity / (2 * Math.PI);

        // Convert radians per second back to rotations per second
        double rotationsPerSecond = radiansPerSecond / (diameter / 2);

        // Convert rotations per second back to rotations per minute
        return rotationsPerSecond * 60.0;
    }

    /**
     * Calculates the shooter speeds required to reach the speaker position.
     * 
     * @param robotPose   the current pose of the robot
     * @param robotSpeeds the current chassis speeds of the robot
     * @return a pair of shooter speeds (left and right) required to reach the
     *         speaker position
     */
    private static Pair<Number, Number> calculateShooterSpeedsForSpeakerApex(Pose2d robotPose, Rotation2d pivotAngle) {
        double desiredRPM = velocityToRPM(ShooterConstants.SPEAKER_V0Z / (pivotAngle.getSin()));
        return Pair.of(desiredRPM, desiredRPM);
    }

    /**
     * Calculates the shooter speeds for passing the apex.
     * 
     * @param robotPose The current pose of the robot.
     * @param pivotAngle The angle of rotation for the pivot.
     * @return A pair of numbers representing the desired RPM for the shooter speeds.
     */
    private static Pair<Number, Number> calculateShooterSpeedsForPassApex(Pose2d robotPose, Rotation2d pivotAngle) {
        double desiredRPM = velocityToRPM(ShooterConstants.PASS_V0Z / (pivotAngle.getSin()));
        return Pair.of(desiredRPM / 1.5, desiredRPM / 1.5);
    }
}