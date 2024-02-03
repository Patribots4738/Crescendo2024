package frc.robot.commands.shooterCalc;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.subsystems.shooter.shooter.Shooter;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.util.SpeedAngleTriplet;

public class ShooterCalc implements Logged, ShooterCalcIO {

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
        return atDesiredAngle().getAsBoolean();
    }

    public boolean shooterAtDesiredRPM() {
        return atDesiredRPM().getAsBoolean();
    }

    public Command stopMotors() {
        return Commands.parallel(
                shooter.stop(),
                pivot.stop());
    }
}