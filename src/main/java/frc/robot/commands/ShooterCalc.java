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
  private BooleanSupplier aiming;

  public ShooterCalc() {
    this.pivot = new Pivot();
    this.shooter = new Shooter();
    this.aiming = (() -> false);
  }

  // Sets needed speed and angle for shooter/pivot relative to speaker or amp based on robot pose
  public Command prepareFireCommand(BooleanSupplier shootAtSpeaker, Pose2d robotPose) {
    SpeedAnglePair pair = calculateSpeed(robotPose, shootAtSpeaker.getAsBoolean());
    return Commands.runOnce(() -> pivot.setAngle(pair.getAngle()))
            .alongWith(Commands.runOnce(() -> shooter.setSpeed(pair.getSpeed())));
  }

  // If we are currently aiming, toggle aiming to false, cancelling repeated calls to prepareFireComman
  // If we are not currently aiming, toggle aiming to true, 
  // and repeatedly run prepareFireCommand until aiming is toggled to false
  // Allows for toggling on and off of aiming with single button
  public Command prepareFireMovingCommand(BooleanSupplier shootAtSpeaker, Swerve swerve) {
    if (aiming.getAsBoolean()) {
      return Commands.runOnce(() -> toggleAiming());
    }
    return Commands.runOnce(() -> toggleAiming())
            .andThen(prepareFireCommand(shootAtSpeaker, swerve.getPose())
                      .repeatedly()
                      .until(() -> !aiming.getAsBoolean()));
  }

  // Sets shooter up to speed without regard to pivot angle
  public Command prepareShooterCommand(BooleanSupplier shootAtSpeaker, Pose2d robotPose) {
    SpeedAnglePair pair = calculateSpeed(robotPose, shootAtSpeaker.getAsBoolean());
    return Commands.runOnce(() -> shooter.setSpeed(pair.getSpeed()));
  }

  // Sets pivot angle without regard to shooter speed
  public Command preparePivotCommand(BooleanSupplier shootAtSpeaker, Pose2d robotPose) {
    SpeedAnglePair pair = calculateSpeed(robotPose, shootAtSpeaker.getAsBoolean());
    return Commands.runOnce(() -> pivot.setAngle(pair.getAngle()));
  }

  // Toggles the aiming BooleanSupplier
  private void toggleAiming() {
    this.aiming = (() -> !aiming.getAsBoolean());
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
}