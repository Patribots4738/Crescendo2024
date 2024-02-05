// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import frc.robot.RobotContainer;
import frc.robot.util.Constants.NTConstants;

public class NoteTrajectory extends Command {
    double x, x0, vx0, ax, z, z0, vz0, az, y, y0, vy0, ay;

    Supplier<Pose2d> poseSupplier;
    Supplier<ChassisSpeeds> speedSupplier;
    Supplier<Pair<Double, Double>> pairSupplier;

    Pose2d initialPose;
    ChassisSpeeds speeds;
    Pair<Double, Double> speedAnglePair;
    Timer timer = new Timer(); 

    Pose3d currentNotePosition = new Pose3d();

    /**
     * Represents a trajectory note.
     * 
     * @param poseSupplier  a supplier for the pose of the trajectory
     * @param speedSupplier a supplier for the chassis speeds of the trajectory
     * @param pairSupplier  a supplier for a pair of doubles representing additional information about the trajectory
     */
    public NoteTrajectory(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier, Supplier<Pair<Double, Double>> pairSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedSupplier = speedSupplier;
        this.pairSupplier = pairSupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialPose = poseSupplier.get();
        speeds = speedSupplier.get();
        speedAnglePair = pairSupplier.get();
        timer.restart();

        x = NTConstants.PIVOT_OFFSET_METERS.getX();
        y = 0;
        z = NTConstants.PIVOT_OFFSET_METERS.getY();
        x0 = NTConstants.PIVOT_OFFSET_METERS.getX();
        y0 = 0;
        z0 = NTConstants.PIVOT_OFFSET_METERS.getY();
        vx0 = Rotation2d.fromDegrees(90 - speedAnglePair.getSecond()).getSin() * speedAnglePair.getFirst()
                + speeds.vxMetersPerSecond;
        vy0 = speeds.vyMetersPerSecond;
        vz0 = Rotation2d.fromDegrees(90 - speedAnglePair.getSecond()).getCos() * speedAnglePair.getFirst();
        ax = 0;
        ay = 0;
        az = -9.8;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double t = timer.get();
        x = kinematicEquation3(x0, vx0, ax, t);
        y = kinematicEquation3(y0, vy0, ay, t);
        z = kinematicEquation3(z0, vz0, az, t);

        currentNotePosition = getNotePose(initialPose, speedAnglePair, x, y, z);

        RobotContainer.components3d[NTConstants.NOTE_INDEX] = getNotePose(initialPose, speedAnglePair, x, y, z).relativeTo(new Pose3d(poseSupplier.get()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        zeroNote();
    }

    // Returns true when either the note is lower than it started
    // or if the timer has reached 3 seconds.
    @Override
    public boolean isFinished() {
        return (z < z0 || timer.get() > 3);
    }

    /**
     * Calculates the pose of a note based on the given parameters. :)
     * 
     * @param pose           The initial pose of the robot.
     * @param speedAnglePair A pair of values representing the speed and angle of
     *                       the note.
     * @param x              The x-coordinate of the note.
     * @param y              The y-coordinate of the note.
     * @param z              The z-coordinate of the note.
     * @return The pose of the note.
     */
    Pose3d getNotePose(Pose2d pose, Pair<Double,Double> speedAnglePair, double x, double y, double z) {
        return new Pose3d(
                new Translation3d(x, y, z).rotateBy(new Rotation3d(
                        0,
                        0,
                        pose.getRotation().getRadians())),
                new Rotation3d())
                .transformBy(
                        new Transform3d(
                                new Translation3d(
                                        pose.getX(),
                                        pose.getY(),
                                        0),
                                new Rotation3d(
                                    Units.degreesToRadians(90), 
                                    -Units.degreesToRadians(speedAnglePair.getSecond()), 
                                    pose.getRotation().getRadians())));
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
    public double kinematicEquation3(double x0, double v0, double a, double t) {
        return x0 + v0 * t + 0.5 * a * Math.pow(t, 2);
    }
}