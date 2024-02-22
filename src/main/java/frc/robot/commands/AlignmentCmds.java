package frc.robot.commands;

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
        return swerve.getDriveCommand(() -> {
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
                () -> alignmentCalc.getAmpAlignmentSpeeds(), 
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
        return swerve.getDriveCommand(() -> alignmentCalc.getChainRotationalSpeeds(driverX.getAsDouble(), driverY.getAsDouble()), () -> true);
    }

    

    public Command sourceRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return swerve.getDriveCommand(() -> alignmentCalc.getSourceRotationalSpeeds(driverX.getAsDouble(), driverY.getAsDouble()), () -> true);
    }

    public Command speakerRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY, ShooterCmds shooterCmds) {
        return swerve.getDriveCommand(
            () -> 
                alignmentCalc.getSpeakerRotationalSpeeds(
                    driverX.getAsDouble(), 
                    driverY.getAsDouble(),
                    shooterCmds), 
            () -> true);
    }

    public Command trapAlignmentCommand(DoubleSupplier driverY) {
        return 
            getAutoAlignmentCommand(
                () -> alignmentCalc.getTrapAlignmentSpeeds(), 
                () -> 
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        -driverY.getAsDouble() * swerve.getPose().getRotation().getCos(),
                        -driverY.getAsDouble() * swerve.getPose().getRotation().getSin(),
                        0,
                        swerve.getPose().getRotation()
                    )
            );
    }

    public Command wingRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return
            Commands.either(
                chainRotationalAlignment(driverX, driverY),
                speakerRotationalAlignment(driverX, driverY, shooterCmds),
                climb::hooksUp);
    }
}
