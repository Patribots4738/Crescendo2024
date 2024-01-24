package frc.robot.commands;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.SpeedAnglePair;

public class ShooterCalc {

  private Pivot pivot;
  private Shooter shooter;
  private boolean aiming;

  public ShooterCalc(Shooter shooter, Pivot pivot) {
    this.pivot = pivot;
    this.shooter = shooter;
    this.aiming = false;
  }



  /**
   * The function prepares a fire command by calculating the speed and angle for the robot's shooter
   * based on the robot's pose and whether it should shoot at the speaker.
   * 
   * @param shootAtSpeaker A BooleanSupplier that returns true if the robot should shoot at the
   * speaker, and false otherwise.
   * @param robotPose The `robotPose` parameter represents the current pose (position and orientation)
   * of the robot. It is of type `Pose2d`.
   * @return The method is returning a Command object.
   */
  public Command prepareFireCommand(BooleanSupplier shootAtSpeaker, Pose2d robotPose) {
    SpeedAnglePair pair = calculateSpeed(robotPose, shootAtSpeaker.getAsBoolean());
    return pivot.setAngleCommand(pair.getAngle())
            .alongWith(shooter.setSpeedCommand(pair.getSpeed()));
  }

  /**
   * The function prepares a command to fire at a stationary target while moving and continuously aiming until
   * instructed to stop.
   * 
   * If we are currently aiming, toggle aiming to false, cancelling repeated calls to prepareFireComman
   * If we are not currently aiming, toggle aiming to true, 
   * and repeatedly run prepareFireCommand until aiming is toggled to false
   * Allows for toggling on and off of aiming with single button
   * 
   * @param shootAtSpeaker A BooleanSupplier that determines whether the robot should shoot at the
   * speaker.
   * @param swerve The "swerve" parameter is an instance of the Swerve class. It is used to access the
   * current pose (position and orientation) of the swerve mechanism.
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
   * The function prepares a shooter command by calculating the speed and angle based on the robot's
   * pose and whether it should shoot at the speaker, and then sets the shooter's speed accordingly.
   * 
   * Sets shooter up to speed without regard to pivot angle
   *
   * @param shootAtSpeaker A BooleanSupplier that returns true if the robot should shoot at the
   * speaker, and false otherwise.
   * @param robotPose The robotPose parameter represents the current pose (position and orientation) of
   * the robot. It is used in the calculateSpeed method to determine the speed at which the shooter
   * should be set.
   * @return The method is returning a Command object.
   */
  public Command prepareShooterCommand(BooleanSupplier shootAtSpeaker, Pose2d robotPose) {
    SpeedAnglePair pair = calculateSpeed(robotPose, shootAtSpeaker.getAsBoolean());
    return shooter.setSpeedCommand(pair.getSpeed());
  }

  /**
   * The function prepares a pivot command based on whether the robot should shoot at the speaker and
   * the current robot pose.
   * 
   * @param shootAtSpeaker A BooleanSupplier that returns true if the robot should shoot at the
   * speaker, and false otherwise.
   * @param robotPose The `robotPose` parameter represents the current pose (position and orientation)
   * of the robot. It is of type `Pose2d`.
   * @return The method is returning a Command object.
   */
  public Command preparePivotCommand(BooleanSupplier shootAtSpeaker, Pose2d robotPose) {
    SpeedAnglePair pair = calculateSpeed(robotPose, shootAtSpeaker.getAsBoolean());
    return pivot.setAngleCommand(pair.getAngle());
  }

  /**
   * The function is a command that resets the shooter to a speed of 0 and an angle constant and 
   * once it has reached its desired states it sets the shooter to a negative speed to pass the 
   * piece back to handoff
   * 
   * @return The method is returning a Command object.
   */
  public Command sendBackCommand() {
    return resetShooter()
              .andThen(Commands.waitUntil(() -> pivot.atDesiredAngle().getAsBoolean() && shooter.atDesiredRPM().getAsBoolean()))
              .andThen(shooter.setSpeedCommand(ShooterConstants.SHOOTER_BACK_SPEED));
  }

  /**
   * Makes aiming false so that we stop any aiming loop currently happening, and then sets the
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

  // Gets a SpeedAnglePair by interpolating values from a map of already 
  // known required speeds and angles for certain poses
  private SpeedAnglePair calculateSpeed(Pose2d robotPose, boolean shootingAtSpeaker) {
        // Constants have blue alliance positions at index 0
        // and red alliance positions at index 1
        int positionIndex = FieldConstants.ALLIANCE == Optional.ofNullable(Alliance.Blue) ? 0 : 1;

        // Get our position relative to the desired field element
        if (shootingAtSpeaker) { 
            robotPose.relativeTo(FieldConstants.SPEAKER_POSITIONS[positionIndex]);
        } else {
            robotPose.relativeTo(FieldConstants.AMP_POSITIONS[positionIndex]);
        }

        double distance = Units.metersToFeet(robotPose.getTranslation().getNorm());
        int lowerIndex = (int) Math.floor(distance / 5.0) * 5;
        int upperIndex = (int) Math.ceil(distance / 5.0) * 5;

        SpeedAnglePair lowerPair = ShooterConstants.SPEAKER_DISTANCES_TO_SPEEDS_AND_ANGLE_MAP.get(lowerIndex);
        SpeedAnglePair upperPair = ShooterConstants.SPEAKER_DISTANCES_TO_SPEEDS_AND_ANGLE_MAP.get(upperIndex);

        return lowerPair.interpolate(upperPair, (distance - lowerIndex) / (upperIndex - lowerIndex));
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
        pivot.stop()
    );
  }
}