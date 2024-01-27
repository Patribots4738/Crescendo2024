// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Neo;
import frc.robot.util.SpeedAnglePair;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    /** Creates a new shooter. */
    private final Neo motorLeft;
    private final Neo motorRight;
    private final Neo pivotMotor;

    public Shooter() {

        motorLeft = new Neo(ShooterConstants.LEFT_SHOOTER_CAN_ID);
        motorRight = new Neo(ShooterConstants.RIGHT_SHOOTER_CAN_ID);

        pivotMotor = new Neo(ShooterConstants.SHOOTER_PIVOT_CAN_ID);

        configMotors();

    }

    public void configMotors() {
        motorLeft.setPID(
                ShooterConstants.SHOOTER_P,
                ShooterConstants.SHOOTER_I,
                ShooterConstants.SHOOTER_D,
                ShooterConstants.SHOOTER_MIN_OUTPUT,
                ShooterConstants.SHOOTER_MAX_OUTPUT);

        motorLeft.addFollower(motorRight, true);
        // Current limit
        // Velocity conversion factor (make gear ratio in constants)

    }

    public Command shoot(Pose2d position, boolean shootingAtSpeaker) {
        SpeedAnglePair pair = calculateSpeed(position, shootingAtSpeaker);
        return runOnce(() -> motorLeft.setTargetPercent(1));
    }

    public Command stop() {
        return runOnce(() -> motorLeft.setTargetPercent(0));
    }

    public SpeedAnglePair calculateSpeed(Pose2d robotPose, boolean shootingAtSpeaker) {
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
        int lowerIndex = (int) (Math.floor(distance / ShooterConstants.MEASUREMENT_INTERVAL_FEET) * ShooterConstants.MEASUREMENT_INTERVAL_FEET);
        int upperIndex = (int) (Math.ceil(distance / ShooterConstants.MEASUREMENT_INTERVAL_FEET) * ShooterConstants.MEASUREMENT_INTERVAL_FEET);
 
        SpeedAnglePair lowerPair = ShooterConstants.SPEAKER_DISTANCES_TO_SPEEDS_AND_ANGLE_MAP.get(lowerIndex);
        SpeedAnglePair upperPair = ShooterConstants.SPEAKER_DISTANCES_TO_SPEEDS_AND_ANGLE_MAP.get(upperIndex);

        return lowerPair.interpolate(upperPair, (distance - lowerIndex) / (upperIndex - lowerIndex));
    }
}
