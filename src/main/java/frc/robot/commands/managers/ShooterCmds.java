package frc.robot.commands.managers;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.logging.NoteTrajectory;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Constants.FieldConstants;
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

    public Command prepareFireCommandAuto(Supplier<Pose2d> robotPose) {
        return Commands.run(() -> {
                desiredTriplet = shooterCalc.calculateSpeakerTriplet(robotPose.get().getTranslation());

                pivot.setAngle(desiredTriplet.getAngle());
                shooter.setSpeed(desiredTriplet.getSpeeds());
            }, pivot, shooter).until(shooterCalc.readyToShootSupplier());
    }

    public void setTriplet(SpeedAngleTriplet triplet) {
        desiredTriplet = triplet;
        pivot.setAngle(triplet.getAngle());
        shooter.setSpeed(triplet.getSpeeds());
    }

    public void setSpeeds(Pair<Double, Double> speeds) {
        shooter.setSpeed(speeds);
    }

    public void setSpeeds(double speed) {
        setSpeeds(Pair.of(speed, speed));
    }

    public Command setTripletCommand(SpeedAngleTriplet triplet) {
        return Commands.run(() -> {
            desiredTriplet = triplet;
            pivot.setAngle(triplet.getAngle());
            shooter.setSpeed(triplet.getSpeeds());
        }, pivot, shooter).until(shooterCalc.readyToShootSupplier());
    }

    public Command sourceIntakeCommand() {
        return Commands.runOnce(() -> {
            desiredTriplet = new SpeedAngleTriplet(-300.0, -300.0, 60.0);
            pivot.setAngle(desiredTriplet.getAngle());
            shooter.setSpeed(desiredTriplet.getSpeeds());
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
        return Commands.run(
            () -> {
                desiredTriplet = shooterCalc.calculateSWDTriplet(robotPose.get(), speeds.get());
                pivot.setAngle(desiredTriplet.getAngle());
                shooter.setSpeed(desiredTriplet.getSpeeds());
            }, pivot, shooter);
    }

    public Command prepareSWDCommandAuto(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds) {
        return Commands.run(
            () -> {
                desiredTriplet = shooterCalc.calculateSWDTripletAuto(robotPose.get(), speeds.get());
                pivot.setAngle(desiredTriplet.getAngle());
                shooter.setSpeed(desiredTriplet.getSpeeds());
            }, pivot, shooter);
    }

    public Command preparePivotCommandAuto(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds) {
        return Commands.run(
            () -> {
                desiredTriplet = shooterCalc.calculateSWDTripletAuto(robotPose.get(), speeds.get());
                pivot.setAngle(desiredTriplet.getAngle());
            }, pivot);
    }

    public Command prepareStillSpeakerCommand(Supplier<Pose2d> robotPose) {
        return prepareSWDCommand(robotPose, () -> new ChassisSpeeds());
    }

    public Command preparePassCommand(Supplier<Pose2d> robotPose, boolean lowPass) {
        return Commands.run(
            () -> {
                desiredTriplet = shooterCalc.calculatePassTriplet(robotPose.get(), lowPass);
                pivot.setAngle(desiredTriplet.getAngle());
                shooter.setSpeed(desiredTriplet.getSpeeds());
            }, pivot, shooter);
    }

    public Command prepareSubwooferCommand() {
        return setTripletCommand(shooterCalc.calculateSpeakerTriplet(FieldConstants.GET_SUBWOOFER_POSITION().getTranslation()));
    }

    public Command getNoteTrajectoryCommand(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        return Commands.runOnce(
            () -> {
                SpeedAngleTriplet calculationTriplet = new SpeedAngleTriplet(shooter.getDesiredSpeeds(), pivot.getTargetAngle());
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
                        shooterCalc.desiredSWDPose,
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

    public Command raisePivot() {
        return pivot.setAngleCommand(ShooterConstants.PIVOT_RAISE_ANGLE_DEGREES);
	}
    
}