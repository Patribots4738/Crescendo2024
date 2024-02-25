package frc.robot.commands.subsytemHelpers;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.commands.sim.note.NoteTrajectory;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.calc.ShooterCalc;
import frc.robot.util.constants.SpeedAngleTriplet;

public class ShooterCmds {

    private Pivot pivot;
    private Shooter shooter;

    private SpeedAngleTriplet desiredTriplet;
    
    public ShooterCalc shooterCalc;

    public ShooterCmds(Shooter shooter, Pivot pivot, ShooterCalc shooterCalc) {
        this.pivot = pivot;
        this.shooter = shooter;
        this.shooterCalc = shooterCalc;
    }
    
    /**
     * Prepares a command for firing the shooter based on the given shooting pose.
     * 
     * @param shootingPose the supplier of the desired shooting pose
     * @return the prepared fire command
     */
    public Command prepareFireCommand(Supplier<Translation2d> shootingPose, BooleanSupplier shouldMirror) {
        return Commands.runEnd(() -> {
                Translation2d desiredPose = shootingPose.get();

                if (shouldMirror.getAsBoolean()) {
                    desiredPose = GeometryUtil.flipFieldPosition(desiredPose);
                }

                desiredTriplet = shooterCalc.calculateTriplet(desiredPose);

                pivot.setAngle(desiredTriplet.getAngle());
                shooter.setSpeed(desiredTriplet.getSpeeds());
            }, 
            () -> {
                if (Robot.gameMode == GameMode.TELEOP) {
                    pivot.setAngle(0);
                    shooter.stop();
                }
            },
            pivot, shooter);
    }

    public Command prepareFireCommandAuto(Supplier<Pose2d> robotPose) {
        return Commands.run(() -> {
                desiredTriplet = shooterCalc.calculateTriplet(robotPose.get().getTranslation());

                pivot.setAngle(desiredTriplet.getAngle());
                shooter.setSpeed(desiredTriplet.getSpeeds());
            }, pivot, shooter);
    }

    public void setTriplet(SpeedAngleTriplet triplet) {
        desiredTriplet = triplet;
        pivot.setAngle(triplet.getAngle());
        shooter.setSpeed(triplet.getSpeeds());
    }

    public Command setTripletCommand(SpeedAngleTriplet triplet) {
        return Commands.run(() -> {
            desiredTriplet = triplet;
            pivot.setAngle(triplet.getAngle());
            shooter.setSpeed(triplet.getSpeeds());
        }, pivot, shooter);
    }
    
    public SpeedAngleTriplet getTriplet() {
        return desiredTriplet;
    }
    
    /**
     * The function prepares a fire command by calculating the speed and angle for
     * the robot's shooter
     * based on the robot's pose and whether it should shoot at the speaker.
     * This should be called with a toggleOnTrue so that it continuously runs on toggle
     * 
     * @param shootAtSpeaker A BooleanSupplier that returns true if the robot should
     *                       shoot at the
     *                       speaker, and false otherwise.
     * @param robotPose      The `robotPose` parameter represents the  supplier of the current pose
     *                       (position and orientation)
     *                       of the robot. It is of type `Supplier<Pose2d>`.
     * @return The method is returning a Command object.
     */
    public Command prepareSWDCommand(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds) {
        return Commands.run(() -> {
            desiredTriplet = shooterCalc.calculateSWDTriplet(robotPose.get(), speeds.get());
            pivot.setAngle(desiredTriplet.getAngle());
            shooter.setSpeed(desiredTriplet.getSpeeds());
        }, pivot, shooter);
    }

    public Command getNoteTrajectoryCommand(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        return Commands.runOnce(
            () -> {
                SpeedAngleTriplet calculationTriplet = shooterCalc.calculateSWDTriplet(pose.get(), speeds.get());

                new NoteTrajectory(
                    pose.get(),
                    speeds.get(),
                    shooterCalc.rpmToVelocity(calculationTriplet.getSpeeds()), 
                    calculationTriplet.getAngle()
                ).andThen(
                    Commands.waitSeconds(.2)
                    .andThen(
                            new NoteTrajectory(
                                pose.get(),
                                speeds.get(),
                                shooterCalc.rpmToVelocity(shooter.getSpeed()), 
                                pivot.getAngle()
                            )
                        )
                ).schedule();
            }
        );
    }

    public Command stopAllMotors() {
        return shooter.stop().andThen(pivot.stop());
    }

}



