// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.util.SpeedAngleTriplet;
import frc.robot.util.Constants.NTConstants;
import monologue.Logged;
import monologue.Annotations.Log;

/**
 * Calculates the trajectory of an object in a 2D plane.
 * 
 * @param endX The x-coordinate at which the trajectory should end. Set to -1 to
 *             calculate until the object reaches the ground.
 * @return An ArrayList of Pose3d objects representing the trajectory of the
 *         object.
 */
public class NoteTrajectory implements Logged {
    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier x0;
    DoubleSupplier y0;
    DoubleSupplier vx0;
    DoubleSupplier vy0;
    DoubleSupplier ax;
    DoubleSupplier ay;

    Supplier<Pose2d> initialPose;

    Timer timer;

    @Log.NT
    Pose3d traj = new Pose3d();

    public NoteTrajectory() {
        setVariables(new Pose2d(), new SpeedAngleTriplet());
        timer = new Timer();
    }

    private void setVariables(Pose2d initialPose2d, SpeedAngleTriplet speedAngleTriplet) {
        x = () -> NTConstants.PIVOT_OFFSET_METERS.getX();
        y = () -> NTConstants.PIVOT_OFFSET_METERS.getY();
        x0 = () -> NTConstants.PIVOT_OFFSET_METERS.getX();
        y0 = () -> NTConstants.PIVOT_OFFSET_METERS.getY();
        // TODO: Change the left and right values to not be the x and y components as
        // they are the wrong axis
        vx0 = () -> Rotation2d.fromDegrees(90 - speedAngleTriplet.getAngle()).getSin()
                * speedAngleTriplet.getLeftSpeed();
        vy0 = () -> Rotation2d.fromDegrees(90 - speedAngleTriplet.getAngle()).getCos()
                * speedAngleTriplet.getLeftSpeed();
        ax = () -> 0;
        ay = () -> -9.8;
        initialPose = () -> initialPose2d;
    }

    public Command getNoteTrajectoryCommand(Supplier<Pose2d> pose, SpeedAngleTriplet speedAngleTriplet) {
        return Commands.runOnce(() -> {
            this.timer.restart();
            setVariables(pose.get(), speedAngleTriplet);
            }).andThen(Commands.runOnce(() -> {
                x = kinematicEquation1(x0, vx0, ax, timer);
                y = kinematicEquation1(y0, vy0, ay, timer);

                traj = getNotePose(this.initialPose, x, y);
                RobotContainer.components3d[NTConstants.NOTE_INDEX] = getNotePose(this.initialPose, x, y).relativeTo(new Pose3d(pose.get()));
            }).repeatedly().until(() -> ((y.getAsDouble() < y0.getAsDouble()) || timer.get() > 5)))
            .andThen(
                Commands.runOnce(
                    this::zeroNote
                )
            );
    }

    Pose3d getNotePose(Supplier<Pose2d> pose, DoubleSupplier x, DoubleSupplier y) {
        return new Pose3d(
                new Translation3d(x.getAsDouble(), 0, y.getAsDouble()).rotateBy(new Rotation3d(
                        0,
                        0,
                        pose.get().getRotation().getRadians())),
                new Rotation3d())
                .transformBy(
                        new Transform3d(
                                new Translation3d(
                                        pose.get().getX(),
                                        pose.get().getY(),
                                        0),
                                new Rotation3d(Units.degreesToRadians(90),0,0)));
    }

    public void zeroNote() {
        RobotContainer.components3d[NTConstants.NOTE_INDEX] = new Pose3d();
    }

    /**
     * Calculates the position using the kinematic equation:
     * x = x0 + v0 * t + 0.5 * a * t^2
     *
     * @param x0 the initial position
     * @param v0 the initial velocity
     * @param a  the acceleration
     * @param t  the time
     * @return the final position
     */
    public DoubleSupplier kinematicEquation1(DoubleSupplier x0, DoubleSupplier v0, DoubleSupplier a, Timer t) {
        DoubleSupplier x = () -> (x0.getAsDouble() + v0.getAsDouble() * t.get() + 0.5 * a.getAsDouble() * Math.pow(t.get(), 2));
        return x;
    }
}
