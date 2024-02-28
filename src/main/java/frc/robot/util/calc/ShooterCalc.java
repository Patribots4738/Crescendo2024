package frc.robot.util.calc;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.constants.SpeedAngleTriplet;
import frc.robot.util.constants.Constants.FieldConstants;
import frc.robot.util.constants.Constants.NTConstants;
import frc.robot.util.constants.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class ShooterCalc implements Logged {
    
    private Shooter shooter;
    private Pivot pivot;

    public ShooterCalc(Shooter shooter, Pivot pivot) {
        this.shooter = shooter;
        this.pivot = pivot;
    }
    
    
    @Log
    double realHeight, gravitySpeedL, gravitySpeedR, gravityAngle;

    @Log
    double distance = 0;

    /**
     * Calculates the shooter speeds required to reach the speaker position.
     * 
     * @param pose   a supplier of the robot's current pose
     * @param speeds a supplier of the robot's current chassis speeds
     * @param dt     the time interval for integration
     * @return a pair of shooter speeds (left and right) required to reach the speaker position
     */
    public SpeedAngleTriplet calculateSWDTriplet(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d currentPose = pose;

        // SpeedAngleTriplet currentTriplet = new SpeedAngleTriplet(calculateShooterSpeedsForApex(pose, calculatePivotAngle(pose)), calculatePivotAngle(pose).getDegrees());
        SpeedAngleTriplet currentTriplet = calculateTriplet(pose.getTranslation());
        double normalVelocity = getVelocityVectorToSpeaker(currentPose, speeds).getY();

        double originalv0 = rpmToVelocity(currentTriplet.getSpeeds());
        double v0z = originalv0 * Math.sin(Units.degreesToRadians(currentTriplet.getAngle()));
        double v0x = originalv0 * Math.cos(Units.degreesToRadians(currentTriplet.getAngle())) + normalVelocity;

        double newv0 = Math.hypot(v0x, v0z);
        Rotation2d newAngle = new Rotation2d(v0x, v0z);

        return 
            SpeedAngleTriplet.of(
                Pair.of(
                    velocityToRPM(newv0),
                    velocityToRPM(newv0)
                ),
                newAngle.getDegrees()
            );
    }

    /**
     * Calculates the pivot angle based on the robot's pose.
     * 
     * @param robotPose The pose of the robot.
     * @return The calculated pivot angle.
     */
    public Rotation2d calculatePivotAngle(Pose2d robotPose) {
        // Calculate the robot's pose relative to the speaker's position
        robotPose = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());

        // Calculate the distance in feet from the robot to the speaker
        double distanceMeters = robotPose.getTranslation().getNorm();

        // Return a new rotation object that represents the pivot angle
        // The pivot angle is calculated based on the speaker's height and the distance to the speaker
        return new Rotation2d(distanceMeters - NTConstants.PIVOT_OFFSET_METERS.getX(), FieldConstants.SPEAKER_HEIGHT_METERS);
    }

    /**
	 * Determines if the pivot rotation is at its target with a small
	 * tolerance
	 * 
	 * @return The method is returning a BooleanSupplier that returns true
	 *         if the pivot is at its target rotation and false otherwise
	 */
	public BooleanSupplier readyToShootSupplier() {
        return () -> pivot.getAtDesiredAngle() && shooter.getAtDesiredRPM();
    }

    // Gets a SpeedAngleTriplet by interpolating values from a map of already
    // known required speeds and angles for certain poses
    public SpeedAngleTriplet calculateTriplet(Translation2d robotPose) {
        // Get our position relative to the desired field element
        // Use the distance as our key for interpolation
        double distanceFeet = Units.metersToFeet(robotPose.getDistance(FieldConstants.GET_SPEAKER_TRANSLATION()));

        this.distance = distanceFeet;

        return ShooterConstants.INTERPOLATION_MAP.get(distanceFeet);
    }
    
    @Log
    Rotation2d currentAngleToSpeaker;
    @Log
    Pose2d desiredSWDPose;
    @Log
    double desiredMPSForNote = 0;
    @Log
    double degreesToSpeakerReferenced = 0;
    @Log
    double angleDifference = 0;

    /**
     * Calculates the angle to the speaker based on the robot's pose and velocity.
     * This is to shoot while driving, but would also work while stationary.
     * 
     * @param robotPose     The current pose of the robot.
     * @param robotVelocity The velocity of the robot.
     * 
     * @return              The angle to the speaker in the form of a Rotation2d object.
     */
    public Rotation2d calculateRobotAngleToSpeaker(Pose2d robotPose, ChassisSpeeds robotVelocity) {
        Translation2d velocityVectorToSpeaker = getVelocityVectorToSpeaker(robotPose, robotVelocity);
        double velocityTangent = velocityVectorToSpeaker.getX();
        // TODO: Check if this velocity should be accounted for in the x component of atan2
        // TODO: I think this should be "newv0" from the SWD calculation for normal velocity
        double velocityNormal = velocityVectorToSpeaker.getY();

        Pose2d poseRelativeToSpeaker = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());
        Rotation2d currentAngleToSpeaker = new Rotation2d(poseRelativeToSpeaker.getX(), poseRelativeToSpeaker.getY());
        double velocityArcTan = Math.atan2(
            velocityTangent,
            // rpmToVelocity(calculateSWDTriplet(robotPose, robotVelocity).getSpeeds())
            rpmToVelocity(calculateShooterSpeedsForApex(robotPose, calculatePivotAngle(robotPose)))
        );
        // Calculate the desired rotation to the speaker, taking into account the tangent velocity
        // Add PI because the speaker opening is the opposite direction that the robot needs to be facing
        Rotation2d desiredRotation2d = Rotation2d.fromRadians(MathUtil.angleModulus(
            currentAngleToSpeaker.getRadians() + velocityArcTan + Math.PI
        )).plus(Rotation2d.fromDegrees(6));

        // Update the robot's pose with the desired rotation
        desiredSWDPose = new Pose2d(robotPose.getTranslation(), desiredRotation2d);

        // Return the desired rotation
        return desiredRotation2d;
    }

    public Rotation2d calculateRobotAngleToSpeaker(Pose2d pose) {
        return calculateRobotAngleToSpeaker(pose, new ChassisSpeeds());
    }

    public Rotation2d calculateRobotAngleToSpeaker(Translation2d translation) {
        return calculateRobotAngleToSpeaker(new Pose2d(translation, new Rotation2d()));
    }

    private Translation2d getVelocityVectorToSpeaker(Pose2d robotPose, ChassisSpeeds robotVelocity) {
        // Calculate the robot's pose relative to the speaker
        Pose2d poseRelativeToSpeaker = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());

        // Calculate the current angle to the speaker
        currentAngleToSpeaker = new Rotation2d(poseRelativeToSpeaker.getX(), poseRelativeToSpeaker.getY());

        // Convert the robot's velocity to a Rotation2d object
        Rotation2d velocityRotation2d = new Rotation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

        // Calculate the total speed of the robot
        double totalSpeed = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

        double angleDifference = velocityRotation2d.getRadians() - currentAngleToSpeaker.getRadians();
        // Calculate the component of the velocity that is tangent to the speaker
        double velocityTangentToSpeaker = totalSpeed * Math.sin(angleDifference);

        double velocityNormalToSpeaker = totalSpeed * Math.cos(angleDifference);
        
        return new Translation2d(velocityTangentToSpeaker, velocityNormalToSpeaker);
    }

    /**
     * This method is averaging the speeds to make a rough estimate of the speed of the note (or the edge of the wheels).
     * The formula used is V = 2π * D/2 * RPM/60.
     * First, it converts from Rotations per Minute to Rotations per Second.
     * Then, it converts from Rotations per Second to Radians per Second.
     * Finally, it multiplies by the radius of the wheel contacting it.
     * As v = rw (w = omega | angular velocity of wheel).
     * 
     * Converts RPM (Revolutions Per Minute) to velocity in meters per second.
     * @param speeds a pair of RPM values representing the speeds of two shooter wheels
     * @return the velocity in meters per second
     */
    public double rpmToVelocity(Pair<Double, Double> speeds) {
        double rotationsPerMinute = (speeds.getFirst() + speeds.getSecond()) / 2.0;
        double rotationsPerSecond = rotationsPerMinute / 60.0;
        double radiansPerSecond = rotationsPerSecond * Math.PI;

        double diameter = ShooterConstants.WHEEL_DIAMETER_METERS;

        desiredMPSForNote = diameter * radiansPerSecond;

        // Normally this is 1 radius * 2 pi
        // but we are doing 2 radius * 1 pi
        // because we are given a diameter
        return diameter * radiansPerSecond;
    }

    /**
     * Converts the velocity of the note to RPM (Rotations Per Minute).
     * Equation: ((V/(2π)) / (D/2)) * 60 = RPM
     * 
     * @param noteVelocity the velocity of the initial note in meters per second
     * @return the RPM (Rotations Per Minute) of the shooter wheel
     */
    public double velocityToRPM(double noteVelocity) {
        double diameter = ShooterConstants.WHEEL_DIAMETER_METERS;
    
        // Convert velocity back to radians per second
        double radiansPerSecond = noteVelocity / (2*Math.PI);
    
        // Convert radians per second back to rotations per second
        double rotationsPerSecond = radiansPerSecond / (diameter/2);
    
        // Convert rotations per second back to rotations per minute
        return rotationsPerSecond * 60.0;
    }

    /**
     * Calculates the shooter speeds required to reach the speaker position.
     * 
     * @param robotPose     the current pose of the robot
     * @param robotSpeeds   the current chassis speeds of the robot
     * @return              a pair of shooter speeds (left and right) required to reach the speaker position
     */
    private Pair<Double, Double> calculateShooterSpeedsForApex(Pose2d robotPose, Rotation2d pivotAngle) {
        double desiredRPM = velocityToRPM(ShooterConstants.V0Z / (pivotAngle.getSin()));
        return Pair.of(desiredRPM, desiredRPM);
    }
}
