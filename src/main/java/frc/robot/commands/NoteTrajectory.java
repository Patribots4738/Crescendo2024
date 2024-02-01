// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
public class NoteTrajectory extends Command implements Logged {
  double x = 0, x0 = 0, vx0 = Rotation2d.fromDegrees(90 - 45).getSin() * 12.0, ax = 0;
  double y = 0, y0 = 0, vy0 = Rotation2d.fromDegrees(90 - 45).getCos() * 12.0, ay = -9.8;
  Timer timer;

  Pose2d initialPose = new Pose2d();

  @Log.NT
  Pose3d traj = new Pose3d();

  // ArrayList<Pose3d> trajList = new ArrayList<Pose3d>();

  /** Creates a new NoteTrajectory. */
  public NoteTrajectory(Supplier<Pose2d> poseSupplier) {
    this.initialPose = poseSupplier.get();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer = new Timer();
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Commands.runOnce(
      () -> {
        double t = timer.get();
        x = kinematicEquation1(x0, vx0, ax, t);
        y = kinematicEquation1(x0, vy0, ay, t);

        traj = getNotePose(initialPose, x, y);
        // trajList.add(getNotePose(initialPose, x, y));
        // traj = trajList.toArray(new Pose3d[1]);
      }
    ).until(() -> isFinished()).asProxy().schedule();   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((y < y0) && timer.get() > 1.5);
  }

  Pose3d getNotePose(Pose2d pose, double x, double y) {
    return new Pose3d(
        new Translation3d(x, 0, y).rotateBy(new Rotation3d(
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
                new Rotation3d()));
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
  public double kinematicEquation1(double x0, double v0, double a, double t) {
    double x = x0 + v0 * t + 0.5 * a * Math.pow(t, 2);
    return x;
  }
}
