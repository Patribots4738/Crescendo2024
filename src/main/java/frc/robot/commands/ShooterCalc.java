package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

                pivot.setAngle(triplet.getAngle());
                shooter.setSpeed(triplet.getSpeeds());
            }, pivot, shooter);
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
    public Command prepareFireMovingCommand(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds) {
        return Commands.run(() -> {
            SpeedAngleTriplet triplet = calculateSWDTriplet(robotPose.get(), speeds.get());
            pivot.setAngle(triplet.getAngle());
            shooter.setSpeed(triplet.getSpeeds());
        }, pivot, shooter);
    }

    public Command prepareFireViaMathCommand(BooleanSupplier shootAtSpeaker, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds) {
        return Commands.run(() -> {
            Rotation2d angle = calculatePivotAngle(robotPose.get());
            Pair<Double, Double> speedsPair = calculateShooterSpeedsForApex(robotPose.get(), angle);

            pivot.setAngle(angle.getDegrees());
            shooter.setSpeed(speedsPair);
        }, pivot, shooter);
    }

    /**
     * Calculates the pivot angle based on the robot's pose.
     * 
     * @param robotPose The pose of the robot.
     * @return The calculated pivot angle.
     */
    public Rotation2d calculatePivotAngle(Pose2d robotPose) {
        // Add the pivot offset to the robot's pose
        robotPose = robotPose.plus(new Transform2d(0.112, 0, robotPose.getRotation()));
        // Calculate the robot's pose relative to the speaker's position
        robotPose = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());

        // Calculate the distance in feet from the robot to the speaker
        double distanceMeters = robotPose.getTranslation().getNorm();

        // Return a new rotation object that represents the pivot angle
        // The pivot angle is calculated based on the speaker's height and the distance to the speaker
        return new Rotation2d(distanceMeters, 3);
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
        Translation2d velocityVectorToSpeaker = getVelocityVectorToSpeaker(robotPose, robotVelocity);
        double velocityTangent = velocityVectorToSpeaker.getX();

        Pose2d poseRelativeToSpeaker = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());
        Rotation2d currentAngleToSpeaker = new Rotation2d(poseRelativeToSpeaker.getX(), poseRelativeToSpeaker.getY());
        double velocityArcTan = Math.atan2(
            velocityTangent,
            rpmToVelocity(calculateSpeed(robotPose, true).getSpeeds())
        );
        // Calculate the desired rotation to the speaker, taking into account the tangent velocity
        // Add PI because the speaker opening is the opposite direction that the robot needs to be facing
        Rotation2d desiredRotation2d = Rotation2d.fromRadians(
            currentAngleToSpeaker.getRadians() + velocityArcTan + Math.PI
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
        double desiredRPM = velocityToRPM(Math.sqrt(2 * 9.8 * 2.08) / (pivotAngle.getSin()));
        return Pair.of(desiredRPM, desiredRPM);
    }

    public Command getNoteTrajectoryCommand(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        return Commands.runOnce(
            () -> {
                Pose2d currentPose = pose.get();
                Rotation2d pivotAngle = calculatePivotAngle(currentPose);
                SpeedAngleTriplet currentTriplet = SpeedAngleTriplet.of(calculateShooterSpeedsForApex(currentPose, pivotAngle), pivotAngle.getDegrees());
                double normalVelocity = -getVelocityVectorToSpeaker(currentPose, speeds.get()).getY();

                double originalv0 = rpmToVelocity(currentTriplet.getSpeeds());
                double v0z = Math.sqrt(9.8*2*2.08);
                double v0x = originalv0 * Math.cos(Math.toRadians(currentTriplet.getAngle())) + normalVelocity;

                double newv0 = Math.sqrt(Math.pow(v0x,2)+ Math.pow(v0z,2));
                Rotation2d newAngle = new Rotation2d(v0x, v0z);

                SpeedAngleTriplet calculationTriplet = 
                    SpeedAngleTriplet.of(
                        Pair.of(
                            velocityToRPM(newv0),
                            velocityToRPM(newv0)
                        ),
                        newAngle.getDegrees()
                    );

                new NoteTrajectory(
                    pose,
                    speeds.get(),
                    rpmToVelocity(calculationTriplet.getSpeeds()), 
                    calculationTriplet.getAngle()
                ).schedule();
            }
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
    private SpeedAngleTriplet calculateSWDTriplet(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d currentPose = pose;
        Rotation2d pivotAngle = calculatePivotAngle(currentPose);
        SpeedAngleTriplet currentTriplet = SpeedAngleTriplet.of(calculateShooterSpeedsForApex(currentPose, pivotAngle), pivotAngle.getDegrees());
        double normalVelocity = -getVelocityVectorToSpeaker(currentPose, speeds).getY();

        double originalv0 = rpmToVelocity(currentTriplet.getSpeeds());
        double v0z = Math.sqrt(9.8*2*2.08);
        double v0x = originalv0 * Math.cos(Math.toRadians(currentTriplet.getAngle())) + normalVelocity;

        double newv0 = Math.sqrt(Math.pow(v0x,2)+ Math.pow(v0z,2));
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
}