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
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.calc.ShooterCalc;
import frc.robot.util.custom.SpeedAngleTriplet;

public class ShooterCmds {

    private Pivot pivot;
    private Shooter shooter;

    private SpeedAngleTriplet desiredTriplet = new SpeedAngleTriplet();
    
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
                    shooter.stopCommand();
                }
            },
            pivot, shooter);
    }

    public Command prepareFireCommandAuto(Supplier<Pose2d> robotPose) {
        return Commands.run(() -> {
                desiredTriplet = shooterCalc.calculateTriplet(robotPose.get().getTranslation());

                pivot.setAngle(desiredTriplet.getAngle());
                shooter.setSpeed(desiredTriplet.getSpeeds());
            }, pivot, shooter).until(shooterCalc.readyToShootSupplier());
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
        }, pivot, shooter).until(shooterCalc.readyToShootSupplier());
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
        return Commands.runEnd(
            () -> {
                desiredTriplet = shooterCalc.calculateSWDTriplet(robotPose.get(), speeds.get());
                pivot.setAngle(desiredTriplet.getAngle());
                shooter.setSpeed(desiredTriplet.getSpeeds());
            },
            () -> {
                if (Robot.gameMode == GameMode.TELEOP) {
                    pivot.setAngle(0);
                    shooter.stopCommand();
                }
            }, pivot, shooter);
    }

    public Command getNoteTrajectoryCommand(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        return Commands.runOnce(
            () -> {
                SpeedAngleTriplet calculationTriplet = new SpeedAngleTriplet(shooter.getTargetSpeeds(), pivot.getTargetAngle());
                SpeedAngleTriplet realTriplet = new SpeedAngleTriplet(shooter.getSpeed(), pivot.getAngle());
                
                Pose2d currentPose = pose.get();
                ChassisSpeeds currentSpeeds = speeds.get();

                // If the shooter is not ready to shoot, 
                // These notes will veer off from each other
                new NoteTrajectory(
                    currentPose,
                    currentSpeeds,
                    shooterCalc.rpmToVelocity(realTriplet.getSpeeds()),
                    realTriplet.getAngle(),
                    true
                ).alongWith(
                    new NoteTrajectory(
                        currentPose,
                        currentSpeeds,
                        shooterCalc.rpmToVelocity(calculationTriplet.getSpeeds()),
                        calculationTriplet.getAngle(),
                        false
                    )
                ).schedule();
            }
        );
    }

    public Command stopShooter() {
        return shooter.stopCommand();
    }

	public Command stowPivot() {
	    return pivot.setAngleCommand(ShooterConstants.PIVOT_LOWER_LIMIT_DEGREES);
	}
}