package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.ShooterConstants;
import frc.robot.util.rev.Neo;
import monologue.Logged;
import monologue.Annotations.Log;

public class Pivot extends SubsystemBase implements Logged {
	private Neo motor;

	@Log
	private double realAngle = 0, desiredAngle = 0;
	
	@Log
	private boolean atDesiredAngle = false;

	public Pivot() {
		motor = new Neo(
            ShooterConstants.SHOOTER_PIVOT_CAN_ID,
            false,
            false, 
            true);
		configMotor();
	}

	public void configMotor() {
		motor.setSmartCurrentLimit(ShooterConstants.PIVOT_CURRENT_LIMIT);
		motor.setPositionConversionFactor(ShooterConstants.PIVOT_POSITION_CONVERSION_FACTOR);
		motor.setPID(ShooterConstants.PIVOT_PID);
        // motor.getAbsoluteEncoder().setZeroOffset(68.8235307-17.6);
	}

	@Override
	public void periodic() {

        realAngle = getAngle();

		atDesiredAngle = 
            MathUtil.isNear(realAngle, desiredAngle, ShooterConstants.PIVOT_DEADBAND);

		RobotContainer.components3d[NTConstants.PIVOT_INDEX] = new Pose3d(
			NTConstants.PIVOT_OFFSET_METERS.getX(),
			0,
			NTConstants.PIVOT_OFFSET_METERS.getZ(),
			new Rotation3d(0, -Units.degreesToRadians(realAngle), 0)
		);
	}

	/**
	 * The function takes an angle in degrees and converts it into a position
	 * for the pivot motor to rotate to
	 * 
	 * @param double The angle to set the shooter to
	 */
	public void setAngle(double angle) {
		angle = MathUtil.clamp(
				angle,
				ShooterConstants.PIVOT_LOWER_LIMIT_DEGREES,
				ShooterConstants.PIVOT_UPPER_LIMIT_DEGREES);

        motor.setTargetPosition(angle);
        desiredAngle = angle;
		
		RobotContainer.desiredComponents3d[NTConstants.PIVOT_INDEX] = new Pose3d(
				NTConstants.PIVOT_OFFSET_METERS.getX(),
				0,
				NTConstants.PIVOT_OFFSET_METERS.getZ(),
				new Rotation3d(0, -Units.degreesToRadians(angle), 0));
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
		return runOnce(() -> setAngle(angle)).andThen(Commands.waitUntil(this::getAtDesiredAngle));
	}

	public double getAngle() {
		return motor.getPosition();
	}

	public double getTargetAngle() {
		return motor.getTargetPosition();
	}

	/**
	 * Determines if the pivot rotation is at its target with a small
	 * tolerance
	 * 
	 * @return The method is returning a BooleanSupplier that returns true
	 *         if the pivot is at its target rotation and false otherwise
	 */
	public boolean getAtDesiredAngle() {
		return atDesiredAngle;
	}
}