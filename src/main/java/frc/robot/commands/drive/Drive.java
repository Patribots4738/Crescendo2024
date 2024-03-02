package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.DriveConstants;

public class Drive extends Command {

    private final Swerve swerve;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldRelativeSupplier;
    private final BooleanSupplier shouldMirror;
    private final BooleanSupplier multiplyXY;
    private final BooleanSupplier multiplyTheta;

    public Drive(
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationsSupplier,
            BooleanSupplier fieldRelativeSupplier,
            BooleanSupplier shouldMirror,
            BooleanSupplier multiplyXY,
            BooleanSupplier multiplyTheta) {

        this.swerve = swerve;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationsSupplier;

        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.shouldMirror = shouldMirror;
        this.multiplyXY = multiplyXY;
        this.multiplyTheta = multiplyTheta;

        addRequirements(swerve);
    }

    public Drive(
            Swerve swerve, 
            Supplier<ChassisSpeeds> speeds, 
            BooleanSupplier fieldRelativeSupplier, 
            BooleanSupplier shouldMirror,
            BooleanSupplier multiplyXY,
            BooleanSupplier multiplyTheta) {

        this.swerve = swerve;

        this.xSupplier = () -> speeds.get().vxMetersPerSecond;
        this.ySupplier = () -> speeds.get().vyMetersPerSecond;
        this.rotationSupplier = () -> speeds.get().omegaRadiansPerSecond;

        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.shouldMirror = shouldMirror;
        this.multiplyXY = multiplyXY;
        this.multiplyTheta = multiplyTheta;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        // The driver's right is negative 
        // on the field's axis
        double y = -ySupplier.getAsDouble();
        double rotation = rotationSupplier.getAsDouble();
        if (shouldMirror.getAsBoolean()) {
            x *= -1;
            y *= -1;
        }
        if (x + y + rotation == 0) {
            swerve.setWheelsX();
        }
        else {
            swerve.drive(
                x * (multiplyXY.getAsBoolean() ? DriveConstants.MAX_SPEED_METERS_PER_SECOND : 1) * swerve.getSpeedMultiplier(),
                y * (multiplyXY.getAsBoolean() ? DriveConstants.MAX_SPEED_METERS_PER_SECOND : 1) * swerve.getSpeedMultiplier(),
                rotation * (multiplyTheta.getAsBoolean() ? DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND : 1) * swerve.getSpeedMultiplier(),
                fieldRelativeSupplier.getAsBoolean());
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
