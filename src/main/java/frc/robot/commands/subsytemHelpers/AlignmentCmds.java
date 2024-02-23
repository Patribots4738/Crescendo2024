package frc.robot.commands.subsytemHelpers;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Swerve;
import frc.robot.util.calc.AlignmentCalc;

public class AlignmentCmds {
    
    private Climb climb;
    private Swerve swerve;
    private ShooterCmds shooterCmds;

    public AlignmentCalc alignmentCalc;

    public AlignmentCmds(Swerve swerve, Climb climb, ShooterCmds shooterCmds) {
        this.climb = climb;
        this.swerve = swerve;
        this.shooterCmds = shooterCmds;
        this.alignmentCalc = new AlignmentCalc(swerve);
    }

    public Command getAutoAlignmentCommand(Supplier<ChassisSpeeds> autoSpeeds, Supplier<ChassisSpeeds> controllerSpeeds) {
        return 
            swerve.getDriveCommand(() -> {
                ChassisSpeeds controllerSpeedsGet = controllerSpeeds.get();
                ChassisSpeeds autoSpeedsGet = autoSpeeds.get();
                return new ChassisSpeeds(
                        (controllerSpeedsGet.vxMetersPerSecond + autoSpeedsGet.vxMetersPerSecond),
                        -(controllerSpeedsGet.vyMetersPerSecond + autoSpeedsGet.vyMetersPerSecond),
                        controllerSpeedsGet.omegaRadiansPerSecond + autoSpeedsGet.omegaRadiansPerSecond);
            }, () -> false);
    }

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

    public Command chainRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getChainRotationalSpeedsSupplier(driverX.getAsDouble(), driverY.getAsDouble()), 
                () -> true
        );
    }    

    /**
     * Command to align the robot rotationally to the source of the field.
     * 
     * @param driverX the driver's x input
     * @param driverY the driver's y input
     * @return        the command to align the robot to the source
     */
    public Command sourceRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getSourceRotationalSpeedsSupplier(driverX.getAsDouble(), driverY.getAsDouble()), 
                () -> true
        );
    }

    public Command speakerRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY, ShooterCmds shooterCmds) {
        return 
            swerve.getDriveCommand(
                alignmentCalc.getSpeakerRotationalSpeedsSupplier(
                    driverX.getAsDouble(),
                    driverY.getAsDouble(),
                    shooterCmds),
                () -> true);
    }

    /**
     * Command to align the robot to the trap of the field.
     * multiplies the driverY by the cosine and sine to get the x and y components of the field relative speed
     * '
     * @param driverY the driver's y input
     * @return        the command to align the robot to the trap
     */
    public Command trapAlignmentCommand(DoubleSupplier driverY) {
        return 
            getAutoAlignmentCommand(
                alignmentCalc.getTrapAlignmentSpeedsSupplier(), 
                () -> 
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        -driverY.getAsDouble() * swerve.getPose().getRotation().getCos(),
                        -driverY.getAsDouble() * swerve.getPose().getRotation().getSin(),
                        0,
                        swerve.getPose().getRotation()
                    )
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
    public Command wingRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return
            Commands.either(
                chainRotationalAlignment(driverX, driverY),
                speakerRotationalAlignment(driverX, driverY, shooterCmds),
                climb::hooksUp);
    }
}
