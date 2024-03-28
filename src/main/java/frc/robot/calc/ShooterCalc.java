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
import frc.robot.Robot;
import frc.robot.logging.NT;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import lib.SpeedAngleTriplet;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
public class ShooterCalc implements Logged {
    
    @IgnoreLogged
    private Shooter shooter;
    @IgnoreLogged
    private Pivot pivot;

    public ShooterCalc(Shooter shooter, Pivot pivot) {
        this.shooter = shooter;
        this.pivot = pivot;
    }
        
    @Log
    double realHeight, gravitySpeedL, gravitySpeedR, gravityAngle;

    /**
     * Calculates the shooter speeds required to reach the speaker position.
     * 
     * @param pose   a supplier of the robot's current pose
     * @param speeds a supplier of the robot's current chassis speeds
     * @param dt     the time interval for integration
     * @return       a pair of shooter speeds (left and right) required to reach the speaker position
     */
    public static SpeedAngleTriplet calculateSWDTriplet(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d currentPose = pose;

        SpeedAngleTriplet currentTriplet = calculateSpeakerTriplet(pose.getTranslation());
        double normalVelocity = PoseCalculations.getVelocityVectorToSpeaker(currentPose, speeds).getY();

        double originalv0 = rpmToVelocity(currentTriplet.getSpeeds());
        double v0z = originalv0 * Math.sin(Units.degreesToRadians(currentTriplet.getAngle()));
        double v0x = originalv0 * Math.cos(Units.degreesToRadians(currentTriplet.getAngle())) + normalVelocity;

        Rotation2d newAngle = new Rotation2d(v0x, v0z);

        return SpeedAngleTriplet.of(
                currentTriplet.getLeftSpeed(),
                currentTriplet.getRightSpeed(),
                newAngle.getDegrees());
    }

    /**
     * Calculates the shooter speeds required to reach the speaker position in auto.
     * 
     * @param pose   a supplier of the robot's current pose
     * @param speeds a supplier of the robot's current chassis speeds
     * @return       a pair of shooter speeds (left and right) required for the note to reach the speaker position
     */
    public SpeedAngleTriplet calculateSWDTripletAuto(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d currentPose = pose;

        SpeedAngleTriplet currentTriplet = calculateSWDTriplet(currentPose, speeds);

        double maxRPMAuto = ShooterConstants.INTERPOLATION_MAP.get(Units.metersToFeet(5)).getAverageSpeed();

        return SpeedAngleTriplet.of(
                MathUtil.clamp(currentTriplet.getLeftSpeed(), 0, maxRPMAuto),
                MathUtil.clamp(currentTriplet.getRightSpeed(), 0, maxRPMAuto),
                currentTriplet.getAngle());
    }

    /**
     * Calculates the shooter speeds required to pass the note to the speaker position.
     * We are literally high tide now.
     * 
     * @param robotPose The pose of the robot.
     * @param speeds    The speeds of the robot.
     * @return          The calculated shooter speeds.
     */
    public SpeedAngleTriplet calculatePassTriplet(Pose2d robotPose, ChassisSpeeds speeds) {
        Rotation2d pivotAngle = calculatePassPivotAngle(robotPose);
        Pair<Number, Number> shooterSpeeds = calculateShooterSpeedsForPassApex(robotPose, pivotAngle);
        return SpeedAngleTriplet.of(
                // Don't ask. It works. Is this how we finally beat the hawaiian kids?
                shooterSpeeds.getFirst(),
                shooterSpeeds.getSecond(),
                pivotAngle.getDegrees());
    }

    /**
     * Calculates the pivot angle required to pass the note to the speaker position.
     * 
     * @param robotPose The pose of the robot.
     * @return          The calculated pivot angle as a Rotation2d object.
     */
    public Rotation2d calculatePassPivotAngle(Pose2d robotPose) {
        // Calculate the robot's pose relative to the speaker's position
        robotPose = robotPose.relativeTo(FieldConstants.GET_PASS_APEX_POSITION());

        // Calculate the distance in feet from the robot to the speaker
        double distanceMeters = robotPose.getTranslation().getNorm();

        // Return a new rotation object that represents the pivot angle
        // The pivot angle is calculated based on the speaker's height and the distance to the speaker
        return new Rotation2d(distanceMeters, FieldConstants.PASS_HEIGHT_METERS + NT.getValue("atan++"));
    }

    public static SpeedAngleTriplet calculateSpeakerTriplet(Translation2d robotPose) {
        // Get our position relative to the desired field element
        // Use the distance as our key for interpolation
        double distanceFeet = Units.metersToFeet(robotPose.getDistance(FieldConstants.GET_SPEAKER_TRANSLATION()));

        return ShooterConstants.INTERPOLATION_MAP.get(distanceFeet);
    }

    @Log
    Rotation2d currentAngleToSpeaker = new Rotation2d();
    @Log
    public Pose2d desiredSWDPose = new Pose2d();
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
    public Rotation2d calculateRobotAngleToPose(Pose2d robotPose, ChassisSpeeds robotVelocity, Pose2d target) {
        Translation2d velocityVectorToSpeaker = getVelocityVectorToSpeaker(robotPose, robotVelocity);
        double velocityTangent = velocityVectorToSpeaker.getX();

        Pose2d poseRelativeToTarget = robotPose.relativeTo(target);
        Rotation2d currentAngleToTarget = new Rotation2d(poseRelativeToTarget.getX(), poseRelativeToTarget.getY());
        double velocityArcTan = Math.atan2(
                velocityTangent,
                target.equals(FieldConstants.GET_PASS_TARGET_POSITION())
                        ? rpmToVelocity(calculatePassTriplet(robotPose, robotVelocity).getSpeeds())
                        : rpmToVelocity(calculateSWDTriplet(robotPose, robotVelocity).getSpeeds())
        // rpmToVelocity(calculateShooterSpeedsForApex(robotPose, calculatePivotAngle(robotPose)))
        );
        // Calculate the desired rotation to the speaker, taking into account the tangent velocity
        // Add PI because the speaker opening is the opposite direction that the robot needs to be facing
        Rotation2d desiredRotation2d = Rotation2d.fromRadians(MathUtil.angleModulus(
                currentAngleToTarget.getRadians() + velocityArcTan + Math.PI));

        // Update the robot's pose with the desired rotation
        desiredSWDPose = new Pose2d(robotPose.getTranslation(), desiredRotation2d);

        // Return the desired rotation
        return desiredRotation2d;
    }

    /**
     * Calculates the angle to the speaker based on the robot's pose.
     * 
     * @param robotPose The current pose of the robot.
     * @return          The angle to the speaker in the form of a Rotation2d object.
     */
    public Rotation2d calculateRobotAngleToPass(Pose2d robotPose) {
        return calculateRobotAngleToPass(robotPose, new ChassisSpeeds());
    }

    /**
     * Calculates the angle to the speaker based on the robot's pose and velocity. 
     * 
     * @param robotPose     The current pose of the robot.
     * @param robotVelocity The velocity of the robot.
     * @return              The angle to the speaker in the form of a Rotation2d object.
     */
    public Rotation2d calculateRobotAngleToPass(Pose2d robotPose, ChassisSpeeds robotVelocity) {
        return calculateRobotAngleToPose(robotPose, robotVelocity, FieldConstants.GET_PASS_TARGET_POSITION());
    }

    /**
     * Calculates the angle to the speaker based on the robot's pose.
     * 
     * @param pose The current pose of the robot.
     * @return     The angle to the speaker in the form of a Rotation2d object.
     */
    public Rotation2d calculateRobotAngleToSpeaker(Pose2d pose) {
        return calculateRobotAngleToSpeaker(pose, new ChassisSpeeds());
    }

    /**
     * Calculates the angle to the speaker based on the robot's pose and velocity.
     * 
     * @param pose          The current pose of the robot.
     * @param robotVelocity The velocity of the robot.
     * @return              The angle to the speaker in the form of a Rotation2d object.
     */
    public Rotation2d calculateRobotAngleToSpeaker(Pose2d pose, ChassisSpeeds robotVelocity) {
        return calculateRobotAngleToPose(pose, robotVelocity, FieldConstants.GET_SPEAKER_POSITION());
    }

    /**
     * Calculates the angle to the speaker based on the robot's translation.
     * 
     * @param translation The current translation of the robot.
     * @return            The angle to the speaker in the form of a Rotation2d object.
     */
    public Rotation2d calculateRobotAngleToSpeaker(Translation2d translation) {
        return calculateRobotAngleToSpeaker(new Pose2d(translation, new Rotation2d()));
    }

    /**
     * Calculates the angle to the speaker based on the robot's pose and velocity.
     * 
     * @param robotPose     The current pose of the robot.
     * @param robotVelocity The velocity of the robot.
     * @return              The angle to the speaker in the form of a Rotation2d object.
     */
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

        if (Robot.isBlueAlliance()) {
            velocityTangentToSpeaker *= -1;
            velocityNormalToSpeaker *= -1;
        }
        
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
        double radiansPerSecond = noteVelocity / (2*Math.PI);
    
        // Convert radians per second back to rotations per second
        double rotationsPerSecond = radiansPerSecond / (diameter/2);
    
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
    private Pair<Number, Number> calculateShooterSpeedsForPassApex(Pose2d robotPose, Rotation2d pivotAngle) {
        double desiredRPM = velocityToRPM(ShooterConstants.PASS_V0Z / (pivotAngle.getSin()));
        return Pair.of(desiredRPM/1.5, desiredRPM/1.5);
    }
}
