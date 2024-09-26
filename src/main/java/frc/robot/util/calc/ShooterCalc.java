package frc.robot.util.calc;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.commands.logging.NT;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.custom.SpeedAngleTriplet;

public class ShooterCalc {
    
    private Shooter shooter;

    private Pivot pivot;

    public ShooterCalc(Shooter shooter, Pivot pivot) {
        this.shooter = shooter;
        this.pivot = pivot;
    }
    
    @AutoLogOutput (key = "Calc/ShooterCalc/RealHeight")
    double realHeight; 
    @AutoLogOutput (key = "Calc/ShooterCalc/GravitySpeedLeft")
    double gravitySpeedL;
    @AutoLogOutput (key = "Calc/ShooterCalc/GravitySpeedRight")
    double gravitySpeedR;
    @AutoLogOutput (key = "Calc/ShooterCalc/GravityAngle")
    double gravityAngle;

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

        SpeedAngleTriplet currentTriplet = calculateSpeakerTriplet(pose.getTranslation());
        double normalVelocity = getVelocityVectorToSpeaker(currentPose, speeds).getY();

        double originalv0 = rpmToVelocity(currentTriplet.getSpeeds());
        double v0z = originalv0 * Math.sin(Units.degreesToRadians(currentTriplet.getAngle()));
        double v0x = originalv0 * Math.cos(Units.degreesToRadians(currentTriplet.getAngle())) + normalVelocity;

        double newv0 = Math.hypot(v0x, v0z);
        Rotation2d newAngle = new Rotation2d(v0x, v0z);

        return 
            SpeedAngleTriplet.of(
                currentTriplet.getLeftSpeed(),
                currentTriplet.getRightSpeed(),
                newAngle.getDegrees()
            );
    }

    public SpeedAngleTriplet calculateSWDTripletAuto(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d currentPose = pose;

        SpeedAngleTriplet currentTriplet = calculateSWDTriplet(currentPose, speeds);

        double maxRPMAuto = ShooterConstants.INTERPOLATION_MAP.get(Units.metersToFeet(5)).getAverageSpeed();
        
        return 
            SpeedAngleTriplet.of(
                MathUtil.clamp(currentTriplet.getLeftSpeed(), 0, maxRPMAuto),
                MathUtil.clamp(currentTriplet.getRightSpeed(), 0, maxRPMAuto),
                currentTriplet.getAngle()
            );
    }

    public SpeedAngleTriplet calculatePassTriplet(Pose2d robotPose) {
        Rotation2d pivotAngle = calculatePassPivotAngle(robotPose);
        Pair<Number, Number> shooterSpeeds = calculateShooterSpeedsForPassApex(robotPose, pivotAngle);
        return SpeedAngleTriplet.of(
            // Don't ask. It works. Is this how we finally beat the hawaiian kids?
            shooterSpeeds.getFirst(),
            shooterSpeeds.getSecond(),
            pivotAngle.getDegrees());
    }

    public SpeedAngleTriplet calculateApexTriplet(Pose2d robotPose) {
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
    public Rotation2d calculatePivotAngle(Pose2d robotPose) {
        // Calculate the robot's pose relative to the speaker's position
        robotPose = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());

        // Calculate the distance in meters from the robot to the speaker
        double distanceMeters = robotPose.getTranslation().getNorm();

        // Return a new rotation object that represents the pivot angle
        // The pivot angle is calculated based on the speaker's height and the distance to the speaker
        return new Rotation2d(
            distanceMeters - NTConstants.PIVOT_OFFSET_METERS.getX(), 
            FieldConstants.SPEAKER_HEIGHT_METERS - NTConstants.PIVOT_OFFSET_METERS.getZ() + NT.getValue("atan++")
        );
    }

    public Rotation2d calculatePassPivotAngle(Pose2d robotPose) {
        // Calculate the robot's pose relative to the speaker's position
        robotPose = robotPose.relativeTo(FieldConstants.GET_PASS_APEX_POSITION());

        // Calculate the distance in feet from the robot to the speaker
        double distanceMeters = robotPose.getTranslation().getNorm();

        // Return a new rotation object that represents the pivot angle
        // The pivot angle is calculated based on the speaker's height and the distance to the speaker
        return new Rotation2d(distanceMeters, FieldConstants.PASS_HEIGHT_METERS + NT.getValue("atan++"));
    }

    /**
	 * Determines if the pivot and shooter is at its target for shooting with a small
	 * tolerance
	 * 
	 * @return The method is returning a BooleanSupplier that returns true
	 *         if the pivot and shooter are at their target states for shooting and false otherwise
	 */
	public BooleanSupplier readyToShootSupplier() {
        return () ->  
            pivot.getAtDesiredAngle()
                && shooter.getAtDesiredRPM()
                && shooter.getAverageTargetSpeed() > 0
                && shooter.getAverageTargetSpeed() != ShooterConstants.DEFAULT_RPM;
    }

    public BooleanSupplier readyToPassSupplier() {
        return () ->
            pivot.getAtDesiredPassAngle()
                && shooter.getAtDesiredPassRPM()
                && shooter.getAverageTargetSpeed() > 0;
    }

    public SpeedAngleTriplet calculateSpeakerTriplet(Translation2d robotPose) {
        // Get our position relative to the desired field element
        // Use the distance as our key for interpolation
        double distanceFeet = Units.metersToFeet(robotPose.getDistance(FieldConstants.GET_SPEAKER_TRANSLATION()));

        return ShooterConstants.INTERPOLATION_MAP.get(distanceFeet);
    }

    @AutoLogOutput (key = "Calc/ShooterCalc/AngleToSpeaker")
    Rotation2d currentAngleToSpeaker = new Rotation2d();
    @AutoLogOutput (key = "Calc/ShooterCalc/DesiredSWDPose")
    public Pose2d desiredSWDPose = new Pose2d();
    @AutoLogOutput (key = "Calc/ShooterCalc/DesiredNoteMPS")
    double desiredMPSForNote = 0;
    @AutoLogOutput (key = "Calc/ShooterCalc/DegreesToSpeakerReferenced")
    double degreesToSpeakerReferenced = 0;
    @AutoLogOutput (key = "Calc/ShooterCalc/AngleDifference")
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
        // TODO: Check if this velocity should be accounted for in the x component of atan2
        // TODO: I think this should be "newv0" from the SWD calculation for normal velocity
        double velocityNormal = velocityVectorToSpeaker.getY();

        Pose2d poseRelativeToTarget = robotPose.relativeTo(target);
        Rotation2d currentAngleToTarget = new Rotation2d(poseRelativeToTarget.getX(), poseRelativeToTarget.getY());
        double velocityArcTan = Math.atan2(
            velocityTangent,
            target.equals(FieldConstants.GET_PASS_TARGET_POSITION()) 
                ? rpmToVelocity(calculatePassTriplet(robotPose).getSpeeds()) 
                : rpmToVelocity(calculateSWDTriplet(robotPose, robotVelocity).getSpeeds())
            // rpmToVelocity(calculateShooterSpeedsForApex(robotPose, calculatePivotAngle(robotPose)))
        );
        // Calculate the desired rotation to the speaker, taking into account the tangent velocity
        // Add PI because the speaker opening is the opposite direction that the robot needs to be facing
        Rotation2d desiredRotation2d = Rotation2d.fromRadians(MathUtil.angleModulus(
            currentAngleToTarget.getRadians() + velocityArcTan + Math.PI
        ));

        // Update the robot's pose with the desired rotation
        desiredSWDPose = new Pose2d(robotPose.getTranslation(), desiredRotation2d);

        // Return the desired rotation
        return desiredRotation2d;
    }

    public Rotation2d calculateRobotAngleToPass(Pose2d robotPose) {
        return calculateRobotAngleToPass(robotPose, new ChassisSpeeds());
    }

    public Rotation2d calculateRobotAngleToPass(Pose2d robotPose, ChassisSpeeds robotVelocity) {
        return calculateRobotAngleToPose(robotPose, robotVelocity, FieldConstants.GET_PASS_TARGET_POSITION());
    }

    public Rotation2d calculateRobotAngleToSpeaker(Pose2d pose) {
        return calculateRobotAngleToSpeaker(pose, new ChassisSpeeds());
    }

    public Rotation2d calculateRobotAngleToSpeaker(Pose2d pose, ChassisSpeeds robotVelocity) {
        return calculateRobotAngleToPose(pose, robotVelocity, FieldConstants.GET_SPEAKER_POSITION());
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
    public double rpmToVelocity(Pair<Double, Double> speeds) {
        double rotationsPerMinute = (speeds.getFirst() + speeds.getSecond()) / 2.0;
        double rotationsPerSecond = rotationsPerMinute / 60.0;
        double radiansPerSecond = rotationsPerSecond * Math.PI;

        double diameter = ShooterConstants.SHOOTER_WHEEL_DIAMETER_METERS;

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
        double diameter = ShooterConstants.SHOOTER_WHEEL_DIAMETER_METERS;
    
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
    private Pair<Number, Number> calculateShooterSpeedsForSpeakerApex(Pose2d robotPose, Rotation2d pivotAngle) {
        double desiredRPM = velocityToRPM(ShooterConstants.SPEAKER_V0Z / (pivotAngle.getSin()));
        return Pair.of(desiredRPM, desiredRPM);
    }

    private Pair<Number, Number> calculateShooterSpeedsForPassApex(Pose2d robotPose, Rotation2d pivotAngle) {
        double desiredRPM = velocityToRPM(ShooterConstants.PASS_V0Z / (pivotAngle.getSin()));
        return Pair.of(desiredRPM/1.5, desiredRPM/1.5);
    }
}
