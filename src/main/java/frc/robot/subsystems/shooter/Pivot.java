package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Neo;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NTConstants;
import frc.robot.util.Constants.ShooterConstants;
import monologue.Logged;
import frc.robot.util.PIDNotConstants;
import monologue.Annotations.Log;

public class Pivot extends SubsystemBase implements Logged {
	private Neo pivot;
    private PIDNotConstants pivotPID;

	@Log
	private double realAngle = 0, desiredAngle = 0;
	
	@Log
	private boolean atDesiredAngle = false;

	public Pivot() {
		pivot = new Neo(
            ShooterConstants.SHOOTER_PIVOT_CAN_ID, 
            !FieldConstants.IS_SIMULATION, 
            true);
		configMotor();
        pivotPID = new PIDNotConstants(ShooterConstants.PIVOT_PID, pivot.getPIDController());
	}

	public void configMotor() {
		pivot.setSmartCurrentLimit(ShooterConstants.PIVOT_CURRENT_LIMIT);
		pivot.setPositionConversionFactor(ShooterConstants.PIVOT_POSITION_CONVERSION_FACTOR);

		pivot.setPID(
            ShooterConstants.PIVOT_PID,
            ShooterConstants.PIVOT_MIN_OUTPUT,
            ShooterConstants.PIVOT_MAX_OUTPUT);
	}

	@Override
	public void periodic() {

        realAngle = -getAngle();

		atDesiredAngle = 
            MathUtil.applyDeadband(
                Math.abs(realAngle + desiredAngle),
                ShooterConstants.PIVOT_DEADBAND) == 0;

		RobotContainer.components3d[NTConstants.PIVOT_INDEX] = new Pose3d(
			NTConstants.PIVOT_OFFSET_METERS.getX(),
			0,
			NTConstants.PIVOT_OFFSET_METERS.getZ(),
			new Rotation3d(0, Units.degreesToRadians(realAngle), 0)
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

        pivot.setTargetPosition(angle);
        desiredAngle = angle;
		
		RobotContainer.desiredComponents3d[NTConstants.PIVOT_INDEX] = new Pose3d(
				NTConstants.PIVOT_OFFSET_METERS.getX(),
				0,
				NTConstants.PIVOT_OFFSET_METERS.getZ(),
				new Rotation3d(0, -Units.degreesToRadians(angle), 0));
	}

    public PIDNotConstants getPIDNotConstants() {
        return this.pivotPID;
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

	public double getAngle() {
		return pivot.getPosition();
	}

	public double getTargetAngle() {
		return pivot.getTargetPosition();
	}

	/**
	 * The function is a command that stops the motor
	 * 
	 * @return The method is returning a Command object.
	 */
	public Command stop() {
		return runOnce(() -> pivot.set(0));
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