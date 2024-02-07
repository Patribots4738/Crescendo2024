package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Neo;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.Neo.TelemetryPreference;
import monologue.Logged;

public class Pivot extends SubsystemBase implements Logged {
	private Neo pivot;

	public Pivot() {
		this.pivot = new Neo(ShooterConstants.SHOOTER_PIVOT_CAN_ID, true);

		configMotor();
	}

	public void configMotor() {
		pivot.setSmartCurrentLimit(ShooterConstants.PIVOT_CURRENT_LIMIT);
		pivot.setTelemetryPreference(TelemetryPreference.ONLY_ABSOLUTE_ENCODER);
		pivot.setPositionConversionFactor(ShooterConstants.PIVOT_POSITION_CONVERSION_FACTOR);

		pivot.setPID(
				ShooterConstants.PIVOT_P,
				ShooterConstants.PIVOT_I,
				ShooterConstants.PIVOT_D,
				ShooterConstants.PIVOT_MIN_OUTPUT,
				ShooterConstants.PIVOT_MAX_OUTPUT);

		// sets brake mode
		pivot.setBrakeMode();
	}

	@Override
	public void periodic() {
		RobotContainer.components3d[NTConstants.PIVOT_INDEX] = new Pose3d(
			NTConstants.PIVOT_OFFSET_METERS.getX(),
			0,
			NTConstants.PIVOT_OFFSET_METERS.getZ(),
			new Rotation3d(0, Units.degreesToRadians(getAngle()), 0)
		);
	}

	/**
	 * The function takes an angle in degrees and converts it into a position
	 * for the pivot motor to rotate to
	 * 
	 * @param double The angle to set the shooter to
	 */
	public void setAngle(double angle) {
		angle = 
			MathUtil.clamp(
				angle, 
				ShooterConstants.PIVOT_LOWER_LIMIT_DEGREES, 
				ShooterConstants.PIVOT_UPPER_LIMIT_DEGREES);
		pivot.setTargetPosition(angle / ShooterConstants.PIVOT_MAX_ANGLE_DEGREES * ShooterConstants.PIVOT_GEAR_RATIO);

		RobotContainer.desiredComponents3d[NTConstants.PIVOT_INDEX] = new Pose3d(
			NTConstants.PIVOT_OFFSET_METERS.getX(),
			0,
			NTConstants.PIVOT_OFFSET_METERS.getZ(),
			new Rotation3d(0, Units.degreesToRadians(angle), 0)
		);
	}

	/**
	 * The function takes an angle in degrees and returns a command that sets
	 * the pivot to the angle converted to a position
	 * 
	 * @param double The angle to set the shooter to
	 * 
	 * @return The method is returning a Command object.
	 */
	public Command setAngleCommand(double angle) {

		return runOnce(() -> setAngle(angle));
	}

	/**
	 * The function sets the pivot angle to the rest angle constant
	 */
	public void setRestAngle() {
		this.setAngle(ShooterConstants.PIVOT_REST_ANGLE_DEGREES);
	}

	/**
	 * The function is a command that sets the rotation of the pivot to
	 * a default resting position
	 * 
	 * @return The method is returning a Command object.
	 */
	public Command setRestAngleCommand() {
		return setAngleCommand(ShooterConstants.PIVOT_REST_ANGLE_DEGREES);
	}

	public double getAngle() {
		return pivot.getPosition() * ShooterConstants.PIVOT_MAX_ANGLE_DEGREES / ShooterConstants.PIVOT_GEAR_RATIO;
	}

	/**
	 * The function is a command that stops the motor
	 * 
	 * @return The method is returning a Command object.
	 */
	public Command stop() {
		return runOnce(() -> pivot.stopMotor());
	}
}