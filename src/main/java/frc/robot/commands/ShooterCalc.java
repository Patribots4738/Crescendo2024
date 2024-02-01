package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.util.SpeedAngleTriplet;

public class ShooterCalc implements Logged{

    private Pivot pivot;
    private Shooter shooter;
    private boolean aiming;


    @Log.NT
    double desiredRSpeed, distance;

    @Log.NT
    double desiredAngle = 0;
    

    public ShooterCalc(Shooter shooter, Pivot pivot) {
        this.pivot = pivot;
        this.shooter = shooter;
        this.aiming = false;
    }

    public boolean getAiming() {
        return aiming;
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
    public Command prepareFireCommand(BooleanSupplier shootAtSpeaker, Supplier<Pose2d> robotPose) {
        return Commands.run(() -> {
                SpeedAngleTriplet triplet = calculateSpeed(robotPose.get(), shootAtSpeaker.getAsBoolean());

                desiredAngle = triplet.getAngle();
                desiredRSpeed = triplet.getRightSpeed();

                // System.out.println(aiming);
        
                pivot.setAngle(desiredAngle);
                shooter.setSpeed(triplet.getSpeeds());
            }, pivot, shooter);
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
        return shooter.stop()
                .alongWith(pivot.setRestAngleCommand());
    }

    // Gets a SpeedAngleTriplet by interpolating values from a map of already
    // known required speeds and angles for certain poses
    public SpeedAngleTriplet calculateSpeed(Pose2d robotPose, boolean shootingAtSpeaker) {
        // Constants have blue alliance positions at index 0
        // and red alliance positions at index 1
        int positionIndex = FieldConstants.ALLIANCE == Optional.ofNullable(Alliance.Blue) ? 0 : 1;

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

    public boolean pivotAtDesiredAngle() {
        return pivot.atDesiredAngle().getAsBoolean();
    }

    public boolean shooterAtDesiredRPM() {
        return shooter.atDesiredRPM().getAsBoolean();
    }

    public Command stopMotors() {
        return Commands.parallel(
                shooter.stop(),
                pivot.stop());
    }
}