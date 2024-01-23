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
  private BooleanSupplier stopAiming;

  public ShooterCalc() {
    this.pivot = new Pivot();
    this.shooter = new Shooter();
    this.stopAiming = (() -> false);
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
    return Commands.runOnce(() -> pivot.setAngle(pair.getAngle()))
            .alongWith(Commands.runOnce(() -> shooter.setSpeed(pair.getSpeed())));
  }

  /**
   * The function prepares a command to fire at a moving target while continuously aiming until
   * instructed to stop.
   * 
   * @param shootAtSpeaker A BooleanSupplier that determines whether the robot should shoot at the
   * speaker.
   * @param swerve The "swerve" parameter is an instance of the Swerve class. It is used to access the
   * current pose (position and orientation) of the swerve mechanism.
   * @return The method is returning a Command object.
   */
  public Command prepareFireMovingCommand(BooleanSupplier shootAtSpeaker, Swerve swerve) {
    if (stopAiming.getAsBoolean()) {
      return Commands.runOnce(() -> toggleStopAiming());
    }
    return Commands.runOnce(() -> toggleStopAiming())
            .andThen(prepareFireCommand(shootAtSpeaker, swerve.getPose())).repeatedly()
            .until(stopAiming);
  }

  /**
   * The function prepares a shooter command by calculating the speed and angle based on the robot's
   * pose and whether it should shoot at the speaker, and then sets the shooter's speed accordingly.
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
    return Commands.runOnce(() -> shooter.setSpeed(pair.getSpeed()));
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
    return Commands.runOnce(() -> pivot.setAngle(pair.getAngle()));
  }

  /**
   * The function toggleStopAiming toggles the value of the stopAiming variable.
   */
  private void toggleStopAiming() {
    this.stopAiming = (() -> !stopAiming.getAsBoolean());
  }

  /**
   * This function calculates the speed and angle needed for a robot to shoot at a speaker or an amp
   * based on its current position on the field.
   * 
   * @param robotPose The `robotPose` parameter is an object of type `Pose2d` which represents the pose
   * (position and orientation) of the robot on the field. It contains information about the robot's
   * translation (position) and rotation (angle) relative to a reference point on the field.
   * @param shootingAtSpeaker A boolean value indicating whether the robot is shooting at the speaker
   * or not.
   * @return The method is returning a SpeedAnglePair object.
   */
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
}