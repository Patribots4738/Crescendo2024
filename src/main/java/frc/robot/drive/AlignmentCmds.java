package frc.robot.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.calc.AlignmentCalc;
import frc.robot.calc.ShooterCalc;
import frc.robot.managers.ShooterCmds;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Swerve;
import monologue.Annotations.IgnoreLogged;

public class AlignmentCmds {
    
    @IgnoreLogged
    private Climb climb;
    @IgnoreLogged
    private Swerve swerve;

    private ShooterCmds shooterCmds;
    @IgnoreLogged
    private AlignmentCalc alignmentCalc;

    public AlignmentCmds(Swerve swerve, Climb climb, ShooterCmds shooterCmds) {
        this.climb = climb;
        this.swerve = swerve;
        this.shooterCmds = shooterCmds;
        this.alignmentCalc = new AlignmentCalc(swerve, shooterCalc);
    }

    /**
     * Command to align the robot rotationally to the source of the field.
     * 
     * @param autoSpeeds       Auto alignment speeds from [-DriveConsants.MAX_SPEED_METERS_PER_SECOND
     *                                                DriveConstants.MAX_SPEED_METERS_PER_SECOND]
     * @param controllerSpeeds Controller speeds from [-1, 1]
     * 
     * @return                 the command to drive the normalized and added autoSpeeds and controllerSpeeds together
     */
    public Command getAutoAlignmentCommand(Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds) {
        return 
            swerve.getDriveCommand(() -> 
                alignmentCalc.normalizeChassisSpeeds(
                    controllerSpeeds.get(),
                    autoSpeeds.get()
                ),
                () -> false);
    }

    /**
     * Command to align the robot to the amp
     * 
     * @param driverX the supplier for the driver controller's x axis
     * @return        the command to auto align to the amp
     */
    public Command ampAlignmentCommand(DoubleSupplier driverX) {
        return 
            getAutoAlignmentCommand(
                alignmentCalc.getAmpAlignmentSpeedsSupplier(), 
                () -> 
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        driverX.getAsDouble() * (Robot.isRedAlliance() ? 1 : -1),
                        0,
                        swerve.getPose().getRotation()
                    )
            );
    }

    /**
     * Command to align the robot rotationally to the nearest chain of the stage.
     * 
     * @param driverX               the supplier for the driver's x input
     * @param driverY               the supplier for the driver's y input
     * @param robotRelativeSupplier the supplier for whether the speeds should be robot relative or not
     * 
     * @return                      the command to align the robot to the nearest chain on the stage
     */
    public Command chainRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY, BooleanSupplier robotRelativeSupplier) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getChainRotationalSpeedsSupplier(driverX, driverY), 
                robotRelativeSupplier
        );
    }    

    /**
     * Command to align the robot rotationally to the source of the field.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * 
     * @return        the command to align the robot to the source
     */
    public Command sourceRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY, BooleanSupplier robotRelativeSupplier) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getSourceRotationalSpeedsSupplier(driverX, driverY), 
                robotRelativeSupplier
        );
    }

    public Command speakerRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY, ShooterCmds shooterCmds) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getSpeakerRotationalSpeedsSupplier(
                    driverX,
                    driverY),
                () -> true);
    }

    public Command passRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY, ShooterCmds shooterCmds) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getPassRotationalSpeedsSupplier(
                    driverX,
                    driverY),
                () -> true);
    }

    /**
     * Command to align the robot to the trap of the field.
     * multiplies the driverY by the cosine and sine to get the x and y components of the field relative speed
     * '
     * @param driverY the driver's y input
     * @return        the command to align the robot to the trap
     */
    public Command trapAlignmentCommand(DoubleSupplier driverX, DoubleSupplier driverY) {
        return 
            getAutoAlignmentCommand(
                alignmentCalc.getTrapAlignmentSpeedsSupplier(), 
                alignmentCalc.getTrapControllerSpeedsSupplier(driverX, driverY)
            );
    }

    /**
     * Command to align the robot to the wing of the field.
     * multiplies the driverY by the cosine and sine to get the x and y components of the field relative speed
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the command to align the robot to the wing
     */
    public Command wingRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY, BooleanSupplier robotRelativeSupplier) {
        return
            Commands.either(
                chainRotationalAlignment(driverX, driverY, robotRelativeSupplier),
                speakerRotationalAlignment(driverX, driverY, shooterCmds)
                    .alongWith(shooterCmds.prepareSWDCommand(swerve::getPose, swerve::getRobotRelativeVelocity)
                        .finallyDo(shooterCmds.stopShooter()::initialize)),
                climb::getHooksUp);
    }

    public Command preparePassCommand(DoubleSupplier driverX, DoubleSupplier driverY, BooleanSupplier robotRelativeSupplier) {
        return
            passRotationalAlignment(driverX, driverY, shooterCmds)
                .alongWith(shooterCmds.preparePassCommand(swerve::getPose, swerve::getRobotRelativeVelocity)
                    .finallyDo(shooterCmds.stopShooter()::initialize));
    }

    public Command preparePresetPose(DoubleSupplier driverX, DoubleSupplier driverY, boolean xButtonPressed) {
        // The true condition represents the pose closer to C1
        // Which is to the left of the driver
        return Commands.either(
            prepareFireAndRotateCommand(driverX, driverY, FieldConstants.ORBIT_POSE),
            prepareFireAndRotateCommand(driverX, driverY, FieldConstants.PODIUM_SHOT_SPOT),
            // No way I just found my first XOR use case :D
            () -> Robot.isRedAlliance() ^ xButtonPressed
        );
    }

    private Command prepareFireAndRotateCommand(DoubleSupplier driverX, DoubleSupplier driverY, Translation2d pose) {
        return shooterCmds.prepareFireCommand(
            () -> pose,
            Robot::isRedAlliance
        ).alongWith(
            swerve.getDriveCommand(() -> {
                Translation2d endingPose = pose;
                if (Robot.isRedAlliance()) {
                    endingPose = GeometryUtil.flipFieldPosition(pose);
                }
                return alignmentCalc.getRotationalSpeeds(
                    driverX.getAsDouble(), 
                    driverY.getAsDouble(), 
                    shooterCalc.calculateRobotAngleToSpeaker(endingPose));
                },
                () -> true
            )
        );
    }
}