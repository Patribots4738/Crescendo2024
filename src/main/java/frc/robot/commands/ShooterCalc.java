package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.SpeedAnglePair;

/**
 * This class represents a command for preparing the shooter based on the
 * robot's pose and target.
 */
public class ShooterCalc extends Command {

    /**
     * Prepares the shooter based on the given parameters.
     * 
     * @param supp      a BooleanSupplier that determines whether to shoot at the
     *                  speaker or the amp
     * @param robotPose the current pose of the robot
     * @param pivot     the pivot object used to set the shooter angle
     * @param shooter   the shooter object used to set the shooter speed
     * @return a Command that prepares the shooter
     */
    public Command prepareShooter(BooleanSupplier supp, Pose2d robotPose, Pivot pivot, Shooter shooter) {
        SpeedAnglePair pair = (supp.getAsBoolean())
                ? getSpeaker(robotPose)
                : getAmp(robotPose);
        return Commands.runOnce(() -> pivot.setAngle(pair.getAngle()))
                .alongWith(Commands.runOnce(() -> shooter.setSpeed(pair.getSpeed())));
    }

    /**
     * Calculates the speed and angle for shooting at the speaker based on the
     * robot's pose.
     * 
     * @param robotPose the current pose of the robot
     * @return a SpeedAnglePair object representing the calculated speed and angle
     */
    private SpeedAnglePair getSpeaker(Pose2d robotPose) {
        return calculateSpeed(robotPose, true);
    }

    /**
     * Calculates the speed and angle for shooting at the amp based on the robot's
     * pose.
     * 
     * @param robotPose the current pose of the robot
     * @return a SpeedAnglePair object representing the calculated speed and angle
     */
    private SpeedAnglePair getAmp(Pose2d robotPose) {
        return calculateSpeed(robotPose, false);
    }

    /**
     * Calculates the speed and angle for shooting based on the robot's pose and
     * target.
     * 
     * @param robotPose         the current pose of the robot
     * @param shootingAtSpeaker a boolean indicating whether the robot is shooting
     *                          at the speaker or the amp
     * @return a SpeedAnglePair object representing the calculated speed and angle
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
        // Our distance is used as our key for interpolation
        // Make sure it is in feet because the interpolation map uses feet
        double distanceFeet = Units.metersToFeet(robotPose.getTranslation().getNorm());

        return ShooterConstants.INTERPOLATION_MAP.get(distanceFeet);
    }
}
