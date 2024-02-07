// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LedStrip;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.LEDConstants;
import frc.robot.util.Constants.FieldConstants.GameMode;
import monologue.Logged;
import monologue.Annotations.Log;

//https://www.tldraw.com/r/borcubmSklQQYMLikl6YZ?viewport=-1638,-855,3094,1889&page=page:page
public class LPI extends Command implements Logged{
    /** Creates a new LPI. */
    private final LedStrip ledStrip;
    private final Supplier<Pose2d> positionSupplier;
    private int activeRotationalLED = 0;

    Pose2d closestPose;
    Pose2d currentRobotPosition;
    Translation2d currentRobotTranslation;
    
    @Log.NT
    Pose2d cardinalMagnitude;

    public LPI(LedStrip ledStrip, Supplier<Pose2d> positionSupplier) {
        this.ledStrip = ledStrip;
        this.positionSupplier = positionSupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override  
    public void execute() {
        closestPose = new Pose2d();
        currentRobotPosition = positionSupplier.get();
        currentRobotTranslation = currentRobotPosition.getTranslation();
        // Loop through all of the startingPositions in the array
        // closest position is point B
        // current position in point A

        double closestDistance = currentRobotTranslation.getDistance(closestPose.getTranslation());

        for (Pose2d startingPosition : LEDConstants.startingPositions) {

            double currentDistance = currentRobotTranslation.getDistance(startingPosition.getTranslation());
            //sets the closest staring position to the closest position
            if (currentDistance < closestDistance) {
                closestPose = startingPosition;
                closestDistance = currentDistance;
            }
        }

        Rotation2d requiredRotation = closestPose.getRotation().minus(currentRobotPosition.getRotation());

        if (closestDistance > LEDConstants.RIN_STAR_BIN) {
            
            Translation2d desiredTranslation = closestPose.relativeTo(currentRobotPosition).getTranslation();

            Rotation2d cardinalDirection = new Rotation2d(desiredTranslation.getX(), desiredTranslation.getY());

            cardinalMagnitude = new Pose2d(desiredTranslation, cardinalDirection);

            int arrowIndex = (int) cardinalDirection.getDegrees()/45;
            //sets the LED to corresponding arrow
            ledStrip.setLED(
                LEDConstants.ARROW_MAP.get((arrowIndex)).getFirst(), 
                LEDConstants.ARROW_MAP.get((arrowIndex)).getSecond(), 
                getColorFromDistance(closestDistance)
            );
        } else if (MathUtil.applyDeadband(requiredRotation.getDegrees(), LEDConstants.LPI_ROTATIONAL_DEADBAND) != 0) {
            ledStrip.setLED(
                activeRotationalLED,
                activeRotationalLED + 2, 
                Color.kGreen);

            activeRotationalLED = requiredRotation.getDegrees() > 0 ? activeRotationalLED + 1 : activeRotationalLED - 1;
            activeRotationalLED %= LEDConstants.LED_COUNT - 2;
        } else {
            // Flash the LEDs based on the current timestamp
            ledStrip.setLED(
                ((int) (Robot.currentTimestamp * 10) % 2 == 0) 
                    ? Color.kGreen 
                    : Color.kGold
            );
        }
    }

    /**
     * Sets the color of the LED strip based on the given distance.
     * If the distance is greater than LEDConstants.OUTER_ZONE, the LED strip is set to red.
     * If the distance is greater than LEDConstants.INNER_ZONE, the LED strip is set to orange.
     * If the distance is greater than LEDConstants.RIN_STAR_BIN, the LED strip is set to green.
     * 
     * @param distance the distance used to determine the LED color
     */
    private Color getColorFromDistance(double distance) {
        if (distance > LEDConstants.OUTER_ZONE) {
            return Color.kRed;
        } else if (distance > LEDConstants.INNER_ZONE) {
            return Color.kOrange;
        } else {
            return Color.kGreen; 
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        cancel();
        ledStrip.greenNGold();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !FieldConstants.GAME_MODE.equals(GameMode.DISABLED);
    }
}
