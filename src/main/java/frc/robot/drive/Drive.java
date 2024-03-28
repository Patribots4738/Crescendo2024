package frc.robot.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot.GameMode;
import frc.robot.subsystems.Swerve;

public class Drive extends Command {

    private final Swerve swerve;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldRelativeSupplier;
    private final BooleanSupplier shouldMirror;
    private double driveMultiplier = 1;

    public Drive(
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationsSupplier,
            BooleanSupplier fieldRelativeSupplier,
            BooleanSupplier shouldMirror) {

        this.swerve = swerve;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationsSupplier;

        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.shouldMirror = shouldMirror;

        addRequirements(swerve);
    }

    public Drive(Swerve swerve, Supplier<ChassisSpeeds> speeds, BooleanSupplier fieldRelativeSupplier, BooleanSupplier shouldMirror) {

        this.swerve = swerve;

        this.xSupplier = () -> speeds.get().vyMetersPerSecond;
        this.ySupplier = () -> speeds.get().vxMetersPerSecond;
        this.rotationSupplier = () -> speeds.get().omegaRadiansPerSecond;

        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.shouldMirror = shouldMirror;

        addRequirements(swerve);
    }

    //Called when the command was initially scheduled.
    @Override
    public void initialize() {
    }

    //Called everytime the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        // The driver's right is negative 
        // on the field's axis
        double y = -ySupplier.getAsDouble();
        double rotation = rotationSupplier.getAsDouble();
        if (shouldMirror.getAsBoolean() || !fieldRelativeSupplier.getAsBoolean()) {
            x *= -1;
            y *= -1;
        }
        if (x + y + rotation == 0 && Robot.gameMode == GameMode.TELEOP) {
            swerve.setWheelsX();
        }
        else {
            swerve.drive(
                x * DriveConstants.MAX_SPEED_METERS_PER_SECOND * driveMultiplier,
                y * DriveConstants.MAX_SPEED_METERS_PER_SECOND * driveMultiplier,
                rotation * DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SECOND * driveMultiplier,
                fieldRelativeSupplier.getAsBoolean());
        }
    }

    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false);
    }

    public void toggleDriveMultiplier() {
        driveMultiplier = (driveMultiplier == 1) ? 0.5 : 1;
    }

    //Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
