package frc.robot.managers;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.calc.ShooterCalc;
import frc.robot.logging.NoteTrajectory;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import lib.SpeedAngleTriplet;
import monologue.Annotations.IgnoreLogged;

public class ShooterCmds {
    
    @IgnoreLogged
    private Pivot pivot;
    @IgnoreLogged
    private Shooter shooter;

    private SpeedAngleTriplet desiredTriplet = new SpeedAngleTriplet();
    
    public ShooterCalc shooterCalc;

    public ShooterCmds(Shooter shooter, Pivot pivot, ShooterCalc shooterCalc) {
        this.pivot = pivot;
        this.shooter = shooter;
        this.shooterCalc = shooterCalc;
    }

    /**
     * Gets the shooter class of the robot
     * 
     * @return the shooter class of the robot
     */
    public Shooter getShooter() {
        return shooter;
    }
    
    /**
     * Prepares a command for firing the shooter based on the given shooting pose.
     * 
     * @param shootingPose the supplier of the desired shooting pose
     * @return the prepared fire command
     */
    public Command prepareFireCommand(Supplier<Translation2d> shootingPose, BooleanSupplier shouldMirror) {
        return Commands.run(() -> {
            Translation2d desiredPose = shootingPose.get();

            if (shouldMirror.getAsBoolean()) {
                desiredPose = GeometryUtil.flipFieldPosition(desiredPose);
            }

            desiredTriplet = shooterCalc.calculateSpeakerTriplet(desiredPose);

            pivot.setAngle(desiredTriplet.getAngle());
            shooter.setSpeed(desiredTriplet.getSpeeds());
        }, pivot, shooter);
    }

    /**
     * Prepares a command for firing the shooter based on the given current pose of the robot.
     * 
     * @param robotPose the supplier of the current pose of the robot
     * @return          the prepared fire command
     */
    public Command prepareFireCommandAuto(Supplier<Pose2d> robotPose) {
        return Commands.run(() -> {
            desiredTriplet = shooterCalc.calculateSpeakerTriplet(robotPose.get().getTranslation());

            pivot.setAngle(desiredTriplet.getAngle());
            shooter.setSpeed(desiredTriplet.getSpeeds());
        }, pivot, shooter).until(shooterCalc.readyToShootSupplier());
    }

    /**
     * Sets the shooter to a specific SpeedAngleTriplet
     * (angle and speeds)
     * @param triplet the SpeedAngleTriplet to set the shooter to
     */
    public void setTriplet(SpeedAngleTriplet triplet) {
        desiredTriplet = triplet;
        pivot.setAngle(triplet.getAngle());
        shooter.setSpeed(triplet.getSpeeds());
    }

    /**
     * Sets the shooter to specific speeds
     * 
     * @param speeds the speeds to set the shooter to
     */
    public void setSpeeds(Pair<Double, Double> speeds) {
        shooter.setSpeed(speeds);
    }

    /**
     * Sets the shooter to a specific speed
     * 
     * @param speed the speed to set the shooter to
     */
    public void setSpeeds(double speed) {
        setSpeeds(Pair.of(speed, speed));
    }

    /**
     * Command to set the shooter to a specific SpeedAngleTriplet
     * 
     * @param triplet the SpeedAngleTriplet to set the shooter to
     * @return        the command to set the shooter to the SpeedAngleTriplet
     */
    public Command setTripletCommand(SpeedAngleTriplet triplet) {
        return Commands.run(() -> {
            desiredTriplet = triplet;
            pivot.setAngle(triplet.getAngle());
            shooter.setSpeed(triplet.getSpeeds());
        }, pivot, shooter).until(shooterCalc.readyToShootSupplier());
    }

    /**
     * Prepares a command to intake the note from the source
     * @return the command to intake the note
     */
    public Command sourceIntakeCommand() {
        return Commands.runOnce(() -> {
            desiredTriplet = new SpeedAngleTriplet(-300.0, -300.0, 60.0);
            pivot.setAngle(desiredTriplet.getAngle());
            shooter.setSpeed(desiredTriplet.getSpeeds());
        }, pivot, shooter);
    }
    
    /**
     * Gets the current SpeedAngleTriplet of the shooter
     * 
     * @return the current SpeedAngleTriplet of the shooter
     */
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
        return Commands.run(
                () -> {
                    desiredTriplet = shooterCalc.calculateSWDTriplet(robotPose.get(), speeds.get());
                    pivot.setAngle(desiredTriplet.getAngle());
                    shooter.setSpeed(desiredTriplet.getSpeeds());
                }, pivot, shooter);
    }

    /**
     * The Command prepares a fire command by calculating the speed and angle for the robot's shooter
     * based on the robot's pose and whether it should shoot at the speaker.
     * 
     * @param robotPose The `robotPose` parameter represents the supplier of the current pose
     * @param speeds    The `speeds` parameter represents the supplier of the current speeds
     * @return          The method is returning a Command object.
     */
    public Command prepareSWDCommandAuto(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds) {
        return Commands.run(
                () -> {
                    desiredTriplet = shooterCalc.calculateSWDTripletAuto(robotPose.get(), speeds.get());
                    pivot.setAngle(desiredTriplet.getAngle());
                    shooter.setSpeed(desiredTriplet.getSpeeds());
                }, pivot, shooter);
    }

    /**
     * The function prepares a pass command by calculating the speed and angle for
     * the robot's shooter
     * based on the robot's pose and whether it should pass the note.
     * 
     * @param robotPose The `robotPose` parameter represents the  supplier of the current pose
     * @param speeds    The `speeds` parameter represents the supplier of the current speeds
     * @return          The method is returning a Command object.
     */
    public Command preparePassCommand(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds) {
        return Commands.run(
                () -> {
                    desiredTriplet = shooterCalc.calculatePassTriplet(robotPose.get(), speeds.get());
                    pivot.setAngle(desiredTriplet.getAngle());
                    shooter.setSpeed(desiredTriplet.getSpeeds());
                }, pivot, shooter);
    }
    
    /**
     * The function prepares a pass command by calculating the speed and angle for
     * the robot's shooter
     * based on the robot's pose and whether it should pass the note.
     * 
     * @param pose   The `pose` parameter represents the supplier of the current pose
     * @param speeds The `speeds` parameter represents the supplier of the current speeds
     * @return       The method is returning a Command object.
     */
    public Command getNoteTrajectoryCommand(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        return Commands.runOnce(
                () -> {
                    SpeedAngleTriplet calculationTriplet = new SpeedAngleTriplet(shooter.getDesiredSpeeds(),
                            pivot.getTargetAngle());
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
                            true).alongWith(
                                    new NoteTrajectory(
                                            shooterCalc.desiredSWDPose,
                                            currentSpeeds,
                                            shooterCalc.rpmToVelocity(calculationTriplet.getSpeeds()),
                                            calculationTriplet.getAngle(),
                                            false))
                            .schedule();
                });
    }

    /**
     * Command to stop the shooter
     * 
     * @return the command to stop the shooter
     */
    public Command stopShooter() {
        return shooter.stopCommand();
    }

    /**
     * Command to put the pivot in the stowed position
     * 
     * @return the command to stow the pivot
     */
	public Command stowPivot() {
        return pivot.setAngleCommand(ShooterConstants.PIVOT_LOWER_LIMIT_DEGREES);
	}
    
}