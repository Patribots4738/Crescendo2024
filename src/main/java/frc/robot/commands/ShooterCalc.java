package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.util.SpeedAngleTriplet;

public class ShooterCalc implements Logged{

    private Pivot pivot;
    private Shooter shooter;


    @Log
    double distance = 0;
    

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
                SpeedAngleTriplet triplet = calculateSpeedTesting(robotPose.get(), shootAtSpeaker.getAsBoolean()); 
        
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
    public Command prepareFireMovingCommand(BooleanSupplier shootAtSpeaker, Supplier<Pose2d> robotPose) {
        return Commands.run(() -> {
                SpeedAngleTriplet triplet = calculateSpeed(robotPose.get(), shootAtSpeaker.getAsBoolean());

                pivot.setAngle(triplet.getAngle());
                shooter.setSpeed(triplet.getSpeeds());
        }, pivot, shooter);
    }


    // Gets a SpeedAngleTriplet by interpolating values from a map of already
    // known required speeds and angles for certain poses
    public SpeedAngleTriplet calculateSpeed(Pose2d robotPose, boolean shootingAtSpeaker) {
        // Constants have blue alliance positions at index 0
        // and red alliance positions at index 1
        int positionIndex = Robot.isBlueAlliance() ? 0 : 1;

        // Get our position relative to the desired field element
        if (shootingAtSpeaker) {
            robotPose = robotPose.relativeTo(FieldConstants.SPEAKER_POSITIONS[positionIndex]);
        } else {
            robotPose = robotPose.relativeTo(FieldConstants.AMP_POSITIONS[positionIndex]);
        }

        // Use the distance as our key for interpolation
        double distanceFeet = Units.metersToFeet(robotPose.getTranslation().getNorm());

        this.distance = robotPose.getX();

        return ShooterConstants.INTERPOLATION_MAP.get(distanceFeet);
    }

    // Gets a SpeedAngleTriplet by interpolating values from a map of already
    // known required speeds and angles for certain poses
    public SpeedAngleTriplet calculateSpeedTesting(Pose2d robotPose, boolean shootingAtSpeaker) {
        // Constants have blue alliance positions at index 0
        // and red alliance positions at index 1
        Rotation2d pivotAngle = calculatePivotAngle(robotPose);
        int positionIndex = Robot.isBlueAlliance() ? 0 : 1;

        // Get our position relative to the desired field element
        if (shootingAtSpeaker) {
            robotPose = robotPose.relativeTo(FieldConstants.SPEAKER_POSITIONS[positionIndex]);
        } else {
            robotPose = robotPose.relativeTo(FieldConstants.AMP_POSITIONS[positionIndex]);
        }

        // Use the distance as our key for interpolation
        double distanceFeet = Units.metersToFeet(robotPose.getTranslation().getNorm());

        this.distance = robotPose.getX();

        SpeedAngleTriplet tempTriplet = ShooterConstants.INTERPOLATION_MAP.get(distanceFeet);
        SpeedAngleTriplet realTriplet = new SpeedAngleTriplet(
            tempTriplet.getFirst(), 
            MathUtil.clamp(pivotAngle.getDegrees(),
                ShooterConstants.PIVOT_LOWER_LIMIT_DEGREES,
                ShooterConstants.PIVOT_UPPER_LIMIT_DEGREES));
        return realTriplet;
    }

    /**
     * Calculates the pivot angle based on the robot's pose.
     * 
     * @param robotPose The pose of the robot.
     * @return The calculated pivot angle.
     */
    public Rotation2d calculatePivotAngle(Pose2d robotPose) {
        // Determine the position index based on the alliance color
        int positionIndex = Robot.isBlueAlliance() ? 0 : 1;

        // Add the pivot offset to the robot's pose
        robotPose = robotPose.plus(new Transform2d(NTConstants.PIVOT_OFFSET_X, 0, new Rotation2d()));

        // Calculate the robot's pose relative to the speaker's position
        robotPose = robotPose.relativeTo(FieldConstants.SPEAKER_POSITIONS[positionIndex]);

        // Calculate the distance in feet from the robot to the speaker
        double distanceMeters = robotPose.getTranslation().getNorm();

        // Return a new rotation object that represents the pivot angle
        // The pivot angle is calculated based on the speaker's height and the distance to the speaker
        return new Rotation2d(distanceMeters, FieldConstants.SPEAKER_HEIGHT_METERS - NTConstants.PIVOT_OFFSET_Z);
    }

    public Command stopMotors() {
        return Commands.parallel(
                shooter.stop(),
                pivot.stop());
    }

    public boolean pivotAtDesiredAngle() {
        return pivot.atDesiredAngle().getAsBoolean();
    }
    
    public boolean shooterAtDesiredRPM() {
        return shooter.atDesiredRPM().getAsBoolean();
    }
}
