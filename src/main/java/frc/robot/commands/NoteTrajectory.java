// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    DoubleSupplier x0;
    DoubleSupplier vx0;
    DoubleSupplier ax;
    DoubleSupplier z;
    DoubleSupplier z0;
    DoubleSupplier vz0;
    DoubleSupplier az;
    DoubleSupplier y;
    DoubleSupplier y0;
    DoubleSupplier vy0;
    DoubleSupplier ay;
    
    Supplier<Pose2d> initialPose;

    @Log.NT
    ChassisSpeeds fieldSpeeds;

    Timer timer;

    @Log.NT
    Pose3d traj = new Pose3d();

    public NoteTrajectory() {
        setVariables(new Pose2d(), new ChassisSpeeds(), new Pair<Double, Double>(0.0, 0.0));
        timer = new Timer();
    }

    private void setVariables(Pose2d initialPose2d, ChassisSpeeds speeds, Pair<Double, Double> speedAnglePair) {
        fieldSpeeds = speeds;
        x = () -> NTConstants.PIVOT_OFFSET_METERS.getX();
        y = () -> 0;
        z = () -> NTConstants.PIVOT_OFFSET_METERS.getY();
        x0 = () -> NTConstants.PIVOT_OFFSET_METERS.getX();
        y0 = () -> 0;
        z0 = () -> NTConstants.PIVOT_OFFSET_METERS.getY();
        // TODO: Change the left and right values to not be the x and y components as
        // they are the wrong axis
        vx0 = () -> Rotation2d.fromDegrees(90 - speedAnglePair.getSecond()).getSin() * speedAnglePair.getFirst()
                    + speeds.vxMetersPerSecond;
        vy0 = () -> speeds.vyMetersPerSecond;
        vz0 = () -> Rotation2d.fromDegrees(90 - speedAnglePair.getSecond()).getCos() * speedAnglePair.getFirst();
        ax = () -> 0;
        ay = () -> 0;
        az = () -> -9.8;

        initialPose = () -> initialPose2d;
    }

    public Command getNoteTrajectoryCommand(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds, Pair<Double, Double> speedAnglePair) {
        return Commands.runOnce(() -> {
            this.timer.restart();
            setVariables(pose.get(), speeds.get(), speedAnglePair);
            }).andThen(Commands.runOnce(() -> {
                x = kinematicEquation1(x0, vx0, ax, timer);
                z = kinematicEquation1(z0, vz0, az, timer);
                y = kinematicEquation1(y0, vy0, ay, timer);

                traj = getNotePose(this.initialPose, speedAnglePair, x, z, y);
                RobotContainer.components3d[NTConstants.NOTE_INDEX] = getNotePose(this.initialPose, speedAnglePair, x, z, y).relativeTo(new Pose3d(pose.get()));
            }).repeatedly().until(() -> ((z.getAsDouble() < z0.getAsDouble()) || timer.get() > 5)))
            .andThen(
                Commands.runOnce(
                    this::zeroNote
                )
            );
    }

    Pose3d getNotePose(Supplier<Pose2d> pose, Pair<Double,Double> speedAnglePair, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z) {
        return new Pose3d(
                new Translation3d(x.getAsDouble(), z.getAsDouble(), y.getAsDouble()).rotateBy(new Rotation3d(
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
                                new Rotation3d(
                                    Units.degreesToRadians(90), 
                                    -Units.degreesToRadians(speedAnglePair.getSecond()), 
                                    pose.get().getRotation().getRadians())));
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
