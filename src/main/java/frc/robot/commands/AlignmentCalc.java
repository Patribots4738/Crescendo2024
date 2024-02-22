package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Swerve;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.constants.Constants.AutoConstants;
import frc.robot.util.constants.Constants.FieldConstants;

public class AlignmentCalc {
    
    private Climb climb;
    private Swerve swerve;
    private ShooterCalc shooterCalc;

    public AlignmentCalc(Swerve swerve, Climb climb, ShooterCalc shooterCalc) {

        this.climb = climb;
        this.swerve = swerve;
        this.shooterCalc = shooterCalc;

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

    public double getAlignmentSpeeds(Rotation2d desiredAngle) {
        return MathUtil.applyDeadband(AutoConstants.HDC.getThetaController().calculate(
            swerve.getPose().getRotation().getRadians(),
            desiredAngle.getRadians()),  0.02);
    }

    public ChassisSpeeds getAmpAlignmentSpeeds() {
        Pose2d ampPose = FieldConstants.GET_AMP_POSITION();
        Pose2d desiredPose = new Pose2d(
            ampPose.getX(),
            swerve.getPose().getY(),
            ampPose.getRotation()
        );
        swerve.setDesiredPose(desiredPose);
        return
            AutoConstants.HDC.calculate(
                swerve.getPose(),
                desiredPose,
                0,
                desiredPose.getRotation()
            );
    }

    public Command ampAlignmentCommand(DoubleSupplier driverX) {
        return 
            getAutoAlignmentCommand(
                () -> getAmpAlignmentSpeeds(), 
                () -> 
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        driverX.getAsDouble() * (Robot.isRedAlliance() ? 1 : -1),
                        0,
                        swerve.getPose().getRotation()
                    )
            );
    }

    public ChassisSpeeds getChainRotationalSpeeds(double driverX, double driverY) {
        Pose2d closestChain = PoseCalculations.getClosestChain(swerve.getPose());
        return new ChassisSpeeds(
            driverY * (Robot.isRedAlliance() ? -1 : 1),
            driverX * (Robot.isRedAlliance() ? -1 : 1),
            getAlignmentSpeeds(closestChain.getRotation())
        );
    }

    public Command chainRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return swerve.getDriveCommand(() -> getChainRotationalSpeeds(driverX.getAsDouble(), driverY.getAsDouble()), () -> true);
    }

    public ChassisSpeeds getSourceRotationalSpeeds(double driverX, double driverY) {
        Pose2d source = FieldConstants.GET_SOURCE_POSITION();
        return new ChassisSpeeds(
            driverY * (Robot.isRedAlliance() ? -1 : 1),
            driverX * (Robot.isRedAlliance() ? -1 : 1),
            getAlignmentSpeeds(source.getRotation())
        );
    }

    public Command sourceRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY) {
        return swerve.getDriveCommand(() -> getSourceRotationalSpeeds(driverX.getAsDouble(), driverY.getAsDouble()), () -> true);
    }

    public ChassisSpeeds getSpeakerRotationalSpeeds(double driverX, double driverY, ShooterCalc shooterCalc) {
        return new ChassisSpeeds(
            driverY * (Robot.isRedAlliance() ? -1 : 1),
            driverX * (Robot.isRedAlliance() ? -1 : 1),
            getAlignmentSpeeds(shooterCalc.calculateSWDRobotAngleToSpeaker(swerve.getPose(), swerve.getFieldRelativeVelocity())));
    }

    public Command speakerRotationalAlignment(DoubleSupplier driverX, DoubleSupplier driverY, ShooterCalc shooterCalc) {
        return swerve.getDriveCommand(
            () -> 
                getSpeakerRotationalSpeeds(
                    driverX.getAsDouble(), 
                    driverY.getAsDouble(),
                    shooterCalc), 
            () -> true);
    }

    public ChassisSpeeds getTrapAlignmentSpeeds() {
        Pose2d closestTrap = PoseCalculations.getClosestChain(swerve.getPose());
        Pose2d stage = FieldConstants.GET_STAGE_POSITION();
        double distance = swerve.getPose().relativeTo(stage).getTranslation().getNorm();
        double x = stage.getX() + distance * closestTrap.getRotation().getCos();
        double y = stage.getY() + distance * closestTrap.getRotation().getSin();
        Pose2d desiredPose = new Pose2d(
            x,
            y,
            closestTrap.getRotation()
        );
        swerve.setDesiredPose(desiredPose);
        return
            AutoConstants.HDC.calculate(
                swerve.getPose(),
                desiredPose,
                0,
                desiredPose.getRotation()
            );
    }

    public Command trapAlignmentCommand(DoubleSupplier driverY) {
        return 
            getAutoAlignmentCommand(
                () -> getTrapAlignmentSpeeds(), 
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
                speakerRotationalAlignment(driverX, driverY, shooterCalc),
                climb::hooksUp);
    }

    public boolean onOppositeSide() {
        return Robot.isRedAlliance() 
            ? swerve.getPose().getX() < FieldConstants.CENTERLINE_X 
            : swerve.getPose().getX() > FieldConstants.CENTERLINE_X;
    }
}
