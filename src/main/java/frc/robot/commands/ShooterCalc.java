package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DriverUI;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.util.SpeedAngleTriplet;

public class ShooterCalc implements Logged {

    private Pivot pivot;
    private Shooter shooter;

    @Log.NT
    double desiredRSpeed = 0, desiredLSpeed = 0, distance = 0, desiredAngle = 0;

    @Log.NT
    double realAngle = 0, realRSpeed = 0, realLSpeed = 0;

    @Log.NT 
    boolean atDesiredAngle = false , atDesiredRPM = false;
    
    
    public ShooterCalc(Shooter shooter, Pivot pivot) {
        this.pivot = pivot;
        this.shooter = shooter;
    }
    
    /**
     * The function prepares a fire command by calculating the speed and angle for
     * the robot's shooter
     * based on the robot's pose and whether it should shoot at the speaker.
     * This should be called with a onTrue
     * 
     * @param shootAtSpeaker A BooleanSupplier that returns true if the robot should
     *                       shoot at the
     *                       speaker, and false otherwise.
     * @param robotPose      The `robotPose` parameter represents the supplier of the current pose
     *                       (position and orientation)
     *                       of the robot. It is of type `Supplier<Pose2d>`.
     * @return The method is returning a Command object.
     */
    public Command prepareFireCommand(BooleanSupplier shootAtSpeaker, Supplier<Pose2d> robotPose) {
        return Commands.runOnce(() -> {
                SpeedAngleTriplet triplet = calculateSpeed(robotPose.get(), shootAtSpeaker.getAsBoolean());

                log(triplet);
        
                pivot.setAngle(triplet.getAngle());
                shooter.setSpeed(triplet.getSpeeds());
            }, pivot, shooter);
    }
    
    @Log.NT
    Rotation2d currentAngleToSpeaker;
    @Log.NT
    Pose2d desiredSWDPose;
    @Log.NT
    double desiredMPSForNote = 0;
    @Log.NT
    double degreesToSpeakerReferenced = 0;


    /**
     * Calculates the angle to the speaker based on the robot's pose and velocity.
     * This is to shoot while driving, but would also work while stationary.
     * 
     * @param robotPose     The current pose of the robot.
     * @param robotVelocity The velocity of the robot.
     * 
     * @return              The angle to the speaker in the form of a Rotation2d object.
     */
    public Rotation2d calculateSWDRobotAngleToSpeaker(Pose2d robotPose, ChassisSpeeds robotVelocity) {
        double velocityTangentToSpeaker = getVelocityVectorToSpeaker(robotPose, robotVelocity).getX();

        // Calculate the desired rotation to the speaker, taking into account the tangent velocity
        Rotation2d desiredRotation2d = Rotation2d.fromRadians(
            currentAngleToSpeaker.getRadians() - Math.atan2(
                velocityTangentToSpeaker, 
                rpmToVelocity(calculateSpeed(robotPose, true).getSpeeds())
            )
        );

        // Update the robot's pose with the desired rotation
        desiredSWDPose = new Pose2d(robotPose.getTranslation(), desiredRotation2d);

        // Return the desired rotation
        return desiredRotation2d;
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
     * Converts the velocity of the shooter wheel to RPM (Rotations Per Minute).
     * Equation: ((V/(2π)) / (D/2)) * 60 = RPM
     * 
     * @param noteVelocity the velocity of the shooter wheel in meters per second
     * @return the RPM (Rotations Per Minute) of the shooter wheel
     */
    public double velocityToRPM(double noteVelocity) {
        double diameter = ShooterConstants.WHEEL_DIAMETER_METERS;
    
        // Convert velocity back to radians per second
        double radiansPerSecond = noteVelocity / diameter;
    
        // Convert radians per second back to rotations per second
        double rotationsPerSecond = radiansPerSecond / Math.PI;
    
        // Convert rotations per second back to rotations per minute
        return rotationsPerSecond * 60.0;
    }

    /**
     * The function prepares a fire command by calculating the speed and angle for
     * the robot's shooter
     * based on the robot's pose and whether it should shoot at the speaker.
     * This should be called with a toggleOnTrue so that it continuously runs on toggle
     * 
     * @param shootAtSpeaker A BooleanSupplier that returns true if the robot should
     *                       shoot at the
     *                       speaker, and false otherwise.
     * @param robotPose      The `robotPose` parameter represents the  supplier of the current pose
     *                       (position and orientation)
     *                       of the robot. It is of type `Supplier<Pose2d>`.
     * @return The method is returning a Command object.
     */
    public Command prepareFireMovingCommand(BooleanSupplier shootAtSpeaker, Supplier<Pose2d> robotPose) {
        return Commands.run(() -> {
                SpeedAngleTriplet triplet = calculateSpeed(robotPose.get(), shootAtSpeaker.getAsBoolean());

                log(triplet);

                pivot.setAngle(triplet.getAngle());
                shooter.setSpeed(triplet.getSpeeds());
            }, pivot, shooter);
    }

    public void logSpeeds(SpeedAngleTriplet triplet) {
        desiredRSpeed = triplet.getRightSpeed();
        desiredLSpeed = triplet.getLeftSpeed();

        realRSpeed = shooter.getSpeed().getSecond();
        realLSpeed = shooter.getSpeed().getFirst();
    }

    public void logAngles(SpeedAngleTriplet triplet) {
        desiredAngle = -triplet.getAngle();
        realAngle = pivot.getAngle();
    }

    public void logAtDesired() {
        atDesiredAngle = pivotAtDesiredAngle();
        atDesiredRPM = shooterAtDesiredRPM();
    }

    public void log(SpeedAngleTriplet triplet) {
        logSpeeds(triplet);
        logAngles(triplet);
        logAtDesired();
    }
    /**
	 * Determines if the pivot rotation is at its target with a small
	 * tolerance
	 * 
	 * @return The method is returning a BooleanSupplier that returns true
	 *         if the pivot is at its target rotation and false otherwise
	 */
	public BooleanSupplier atDesiredAngle() {
		return () -> (MathUtil.applyDeadband(
				Math.abs(
						pivot.getAngle() - desiredAngle),
				ShooterConstants.PIVOT_DEADBAND) == 0);
	}

    // TODO: Implement a way to get the RPM of the shooter
    /**
     * The function is a BooleanSupplier that represents the the condition of
     * the velocity of the motor being equal to its targetVelocity
     * 
     * 
     * @return The method is returning a BooleanSupplier that returns true if
     *         the current velocity of the motors is at the target velocity with a
     *         small tolerance
     */
    public BooleanSupplier atDesiredRPM() {
        return () -> (MathUtil.applyDeadband(
                Math.abs(
                        shooter.getSpeed().getFirst() - shooter.getSpeed().getFirst()),
                ShooterConstants.SHOOTER_DEADBAND) == 0
                && MathUtil.applyDeadband(
                Math.abs(
                        shooter.getSpeed().getSecond() - shooter.getSpeed().getSecond()),
                ShooterConstants.SHOOTER_DEADBAND) == 0);
    }

    /**
     * The function is a command that resets the shooter to a speed of 0 and an
     * angle constant and
     * once it has reached its desired states it sets the shooter to a negative
     * speed to pass the
     * piece back to handoff
     * 
     * @return The method is returning a Command object.
     */
    public Command sendBackCommand() {
        return resetShooter()
                .andThen(Commands.waitUntil(
                        () -> atDesiredAngle().getAsBoolean() && atDesiredRPM().getAsBoolean()))
                .andThen(shooter.setSpeedCommand(ShooterConstants.SHOOTER_BACK_SPEED));
    }

    /**
     * Makes aiming false so that we stop any aiming loop currently happening, and
     * then sets the
     * shooter to a speed of 0 and the pivot angle to a predetermined constant
     * 
     * @return The method is returning a Command object.
     */
    public Command resetShooter() {
        return shooter.stop()
                .alongWith(pivot.setRestAngleCommand());
    }

    // Gets a SpeedAngleTriplet by interpolating values from a map of already
    // known required speeds and angles for certain poses
    public SpeedAngleTriplet calculateSpeed(Pose2d robotPose, boolean shootingAtSpeaker) {
        // Constants have blue alliance positions at index 0
        // and red alliance positions at index 1
        int positionIndex = FieldConstants.IS_BLUE_ALLIANCE() ? 0 : 1;

        // Get our position relative to the desired field element
        if (shootingAtSpeaker) {
            robotPose = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());
        } else {
            robotPose = robotPose.relativeTo(FieldConstants.GET_AMP_POSITION());
        }

        // Use the distance as our key for interpolation
        double distanceFeet = Units.metersToFeet(robotPose.getTranslation().getNorm());

        this.distance = robotPose.getX();

        return ShooterConstants.INTERPOLATION_MAP.get(distanceFeet);
    }

    /**
     * Checks if the pivot is at the desired angle.
     * 
     * @return true if the pivot is at the desired angle, false otherwise.
     */
    public boolean pivotAtDesiredAngle() {
        return atDesiredAngle().getAsBoolean();
    }

    /**
     * Checks if the shooter is at the desired RPM.
     * 
     * @return true if the shooter is at the desired RPM, false otherwise
     */
    public boolean shooterAtDesiredRPM() {
        return atDesiredRPM().getAsBoolean();
    }

    /**
     * Stops the motors of the shooter and pivot subsystems.
     * 
     * @return The command to stop the motors.
     */
    public Command stopMotors() {
        return Commands.parallel(
                shooter.stop(),
                pivot.stop());
    }

    public Command getNoteTrajectoryCommand(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        SpeedAngleTriplet calculationTriplet = calculateSWDSpeedAngleTripletToSpeaker(pose, speeds);

        return Commands.runOnce(() -> new NoteTrajectory(
            pose, 
            speeds, 
            () -> Pair.of(
                rpmToVelocity(calculationTriplet.getSpeeds()), 
                calculationTriplet.getAngle())
        ).schedule());
    }
    
    private SpeedAngleTriplet calculateSWDSpeedAngleTripletToSpeaker(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds){
        double dt = DriverUI.currentTimestamp - DriverUI.previousTimestamp;

        return new SpeedAngleTriplet(
            calculateSWDShooterSpeedsToSpeaker(pose, speeds, dt),
            calculateSWDPivotAngleToSpeaker(pose, speeds, dt).getDegrees()
        );
    }

    /**
     * Calculates the shooter speeds required to reach the speaker position.
     * 
     * @param pose   a supplier of the robot's current pose
     * @param speeds a supplier of the robot's current chassis speeds
     * @param dt     the time interval for integration
     * @return a pair of shooter speeds (left and right) required to reach the speaker position
     */
    private Pair<Double, Double> calculateSWDShooterSpeedsToSpeaker(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds, double dt) {
        Pose2d estimatedPose = integratePosition(pose, speeds, dt);

        SpeedAngleTriplet futureTriplet = 
            ShooterConstants.INTERPOLATION_MAP.get(
                estimatedPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION()).getX()
            );
            

        double velocityNormalToSpeaker = getVelocityVectorToSpeaker(pose.get(), speeds.get()).getY();
        double averageAddedRPM = velocityToRPM(velocityNormalToSpeaker);

        double averageTotalRPM = futureTriplet.getAverageSpeed() - averageAddedRPM;

        // Un-average the rpm to get the shooter wheels up to speed
        // Use the difference at the initial guess to get a gooder average
        double difference = futureTriplet.getLeftSpeed() - futureTriplet.getRightSpeed();

        return Pair.of(
            averageTotalRPM + difference, 
            averageTotalRPM - difference
        );
    }


    private Rotation2d calculateSWDPivotAngleToSpeaker(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds, double dt) {
        Pose2d estimatedPose = integratePosition(pose, speeds, dt);

        estimatedPose = estimatedPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());

        return Rotation2d.fromDegrees(ShooterConstants.INTERPOLATION_MAP.get(estimatedPose.getX()).getAngle());
    }

    private Pose2d integratePosition(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds, double dt) {
        Pose2d robotPose = pose.get();
        ChassisSpeeds robotSpeeds = speeds.get();
        
        Pose2d estimatedPose = robotPose.exp(new Twist2d(
            robotSpeeds.vxMetersPerSecond * dt, 
            robotSpeeds.vyMetersPerSecond * dt, 
            robotSpeeds.omegaRadiansPerSecond * dt)
        );
        return estimatedPose;
    }
}