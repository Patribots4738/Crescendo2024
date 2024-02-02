package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.util.SpeedAngleTriplet;

public class ShooterCalc implements Logged {

    private Pivot pivot;
    private Shooter shooter;
    private boolean aiming;

    private NoteTrajectory noteTrajectory;
    
    public ShooterCalc(Shooter shooter, Pivot pivot) {
        this.pivot = pivot;
        this.shooter = shooter;
        this.aiming = false;
        this.noteTrajectory = noteTrajectory;
    }
    
    /**
     * The function prepares a fire command by calculating the speed and angle for
     * the robot's shooter
     * based on the robot's pose and whether it should shoot at the speaker.
     * 
     * @param shootAtSpeaker A BooleanSupplier that returns true if the robot should
     *                       shoot at the
     *                       speaker, and false otherwise.
     * @param robotPose      The `robotPose` parameter represents the current pose
     *                       (position and orientation)
     *                       of the robot. It is of type `Pose2d`.
     * @return The method is returning a Command object.
     */
    public Command prepareFireCommand(BooleanSupplier shootAtSpeaker, Pose2d robotPose) {
        SpeedAngleTriplet triplet = calculateSpeed(robotPose, shootAtSpeaker.getAsBoolean());
        
        return pivot.setAngleCommand(triplet.getAngle())
            .alongWith(shooter.setSpeedCommand(triplet.getSpeeds()));
    }
    
    @Log.NT
    Rotation2d currentAngleToSpeaker;
    @Log.NT
    Pose2d robotPoseAngled;
    @Log.NT
    Pose2d robotPoseSurely;


    /**
     * Calculates the angle to the speaker based on the robot's pose and velocity.
     * This is to shoot while driving, but would also work while stationary.
     * 
     * @param robotPose     The current pose of the robot.
     * @param robotVelocity The velocity of the robot.
     * 
     * @return              The angle to the speaker in the form of a Rotation2d object.
     */
    public Rotation2d calculateSWDAngleToSpeaker(Pose2d robotPose, ChassisSpeeds robotVelocity) {
        Pose2d poseRelativeToSpeaker = robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());
        currentAngleToSpeaker = new Rotation2d(poseRelativeToSpeaker.getX(), poseRelativeToSpeaker.getY());
        robotPoseAngled = new Pose2d(robotPose.getTranslation(), currentAngleToSpeaker);

        Translation2d velocityTranslation = new Translation2d(
                Math.cos(currentAngleToSpeaker.getRadians())
                        * rpmToVelocity(calculateSpeed(robotPoseAngled, true).getSpeeds()),
                Math.signum(robotVelocity.vyMetersPerSecond)
                        * Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
                        - Math.sin(currentAngleToSpeaker.getRadians())
                                * rpmToVelocity(calculateSpeed(robotPoseAngled, true).getSpeeds()));

        double desiredMPSForNote = Math.hypot(velocityTranslation.getX(), velocityTranslation.getY());

        robotPoseSurely = new Pose2d(robotPose.getTranslation(), new Rotation2d(velocityTranslation.getX(), velocityTranslation.getY()).unaryMinus());

        return new Rotation2d(velocityTranslation.getX(), velocityTranslation.getY());
    }

    /**
     * TODO: These should both probably work off of the current speed, not the desired speed.
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
     * The function prepares a command to fire at a stationary target while moving
     * and continuously aiming until
     * instructed to stop.
     * 
     * If we are currently aiming, toggle aiming to false, cancelling repeated calls
     * to prepareFireCommand
     * If we are not currently aiming, toggle aiming to true,
     * and repeatedly run prepareFireCommand until aiming is toggled to false
     * Allows for toggling on and off of aiming with single button
     * 
     * @param shootAtSpeaker A BooleanSupplier that determines whether the robot
     *                       should shoot at the
     *                       speaker.
     * @param swerve         The "swerve" parameter is an instance of the Swerve
     *                       class. It is used to access the
     *                       current pose (position and orientation) of the swerve
     *                       mechanism.
     * @return The method is returning a Command object.
     */
    public Command prepareFireMovingCommand(BooleanSupplier shootAtSpeaker, Swerve swerve) {
        if (aiming) {
            return Commands.runOnce(() -> toggleAiming());
        }
        return Commands.runOnce(() -> toggleAiming())
                .andThen(prepareFireCommand(shootAtSpeaker, swerve.getPose())
                        .repeatedly()
                        .until(() -> !aiming));
    }

    /**
     * The function prepares a shooter command by calculating the speed and angle
     * based on the robot's
     * pose and whether it should shoot at the speaker, and then sets the shooter's
     * speed accordingly.
     * 
     * Sets shooter up to speed without regard to pivot angle
     *
     * @param shootAtSpeaker A BooleanSupplier that returns true if the robot should
     *                       shoot at the
     *                       speaker, and false otherwise.
     * @param robotPose      The robotPose parameter represents the current pose
     *                       (position and orientation) of
     *                       the robot. It is used in the calculateSpeed method to
     *                       determine the speed at which the shooter
     *                       should be set.
     * @return The method is returning a Command object.
     */
    public Command prepareShooterCommand(BooleanSupplier shootAtSpeaker, Pose2d robotPose) {
        SpeedAngleTriplet triplet = calculateSpeed(robotPose, shootAtSpeaker.getAsBoolean());
        return shooter.setSpeedCommand(triplet.getSpeeds());
    }

    /**
     * The function prepares a pivot command based on whether the robot should shoot
     * at the speaker and
     * the current robot pose.
     * 
     * @param shootAtSpeaker A BooleanSupplier that returns true if the robot should
     *                       shoot at the
     *                       speaker, and false otherwise.
     * @param robotPose      The `robotPose` parameter represents the current pose
     *                       (position and orientation)
     *                       of the robot. It is of type `Pose2d`.
     * @return The method is returning a Command object.
     */
    public Command preparePivotCommand(BooleanSupplier shootAtSpeaker, Pose2d robotPose) {
        SpeedAngleTriplet triplet = calculateSpeed(robotPose, shootAtSpeaker.getAsBoolean());
        return pivot.setAngleCommand(triplet.getAngle());
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
                        () -> pivot.atDesiredAngle().getAsBoolean() && shooter.atDesiredRPM().getAsBoolean()))
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
        return Commands.runOnce(() -> stopAiming())
                .andThen(shooter.stop()
                        .alongWith(pivot.setRestAngleCommand()));
    }

    // Toggles the aiming boolean
    private void toggleAiming() {
        this.aiming = !aiming;
    }

    // Sets the aiming boolean to false
    private void stopAiming() {
        this.aiming = false;
    }

    // Sets the aiming boolean to true
    private void startAiming() {
        this.aiming = true;
    }

    // Gets a SpeedAngleTriplet by interpolating values from a map of already
    // known required speeds and angles for certain poses
    private SpeedAngleTriplet calculateSpeed(Pose2d robotPose, boolean shootingAtSpeaker) {
        // Get our position relative to the desired field element
        if (shootingAtSpeaker) {
            robotPose.relativeTo(FieldConstants.GET_SPEAKER_POSITION());
        } else {
            robotPose.relativeTo(FieldConstants.GET_AMP_POSITION());
        }

        // Use the distance as our key for interpolation
        double distanceFeet = Units.metersToFeet(robotPose.getTranslation().getNorm());

        return ShooterConstants.INTERPOLATION_MAP.get(distanceFeet);
    }

    /**
     * Checks if the pivot is at the desired angle.
     * 
     * @return true if the pivot is at the desired angle, false otherwise.
     */
    public boolean pivotAtDesiredAngle() {
        return pivot.atDesiredAngle().getAsBoolean();
    }

    /**
     * Checks if the shooter is at the desired RPM.
     * 
     * @return true if the shooter is at the desired RPM, false otherwise
     */
    public boolean shooterAtDesiredRPM() {
        return shooter.atDesiredRPM().getAsBoolean();
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

    public Command getNoteTrajectoryCommand(Supplier<Pose2d> pose, SpeedAngleTriplet speedAngleTriplet) {
        return noteTrajectory.getNoteTrajectoryCommand(pose, speedAngleTriplet);
    }
}