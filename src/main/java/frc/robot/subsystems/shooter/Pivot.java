package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

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
import frc.robot.util.Neo.TelemetryPreference;
import monologue.Logged;
import frc.robot.util.PIDNotConstants;
import monologue.Annotations.Log;

public class Pivot extends SubsystemBase implements Logged {
	private Neo pivot;
    private PIDNotConstants pivotPID;
	private AbsoluteEncoder pivotEncoder;
	private SparkPIDController pivotPIDController;

	@Log
	public double realAngle = 0, desiredAngle = 0;
	
	@Log
	public boolean atDesiredAngle = false;

	public Pivot() {
		this.pivot = new Neo(ShooterConstants.SHOOTER_PIVOT_CAN_ID);
		this.pivotEncoder = pivot.getAbsoluteEncoder(Type.kDutyCycle);
		this.pivotPIDController = pivot.getPIDController();
		configMotor();
        pivotPID = new PIDNotConstants(pivot.getPID(), pivot.getPIDController());
	}

	public void configMotor() {
        pivotEncoder.setInverted(true);
		pivot.setSmartCurrentLimit(ShooterConstants.PIVOT_CURRENT_LIMIT);
		pivot.setTelemetryPreference(TelemetryPreference.ONLY_ABSOLUTE_ENCODER);
		pivotPIDController.setFeedbackDevice(pivotEncoder);
		pivotEncoder.setPositionConversionFactor(ShooterConstants.PIVOT_POSITION_CONVERSION_FACTOR);

		pivot.setPID(
				ShooterConstants.PIVOT_P,
				ShooterConstants.PIVOT_I,
				ShooterConstants.PIVOT_D,
				ShooterConstants.PIVOT_MIN_OUTPUT,
				ShooterConstants.PIVOT_MAX_OUTPUT);
	}

	@Override
	public void periodic() {

        if (FieldConstants.IS_SIMULATION) {
            if (Math.abs(desiredAngle - realAngle) > 3) {
                realAngle += (desiredAngle - realAngle) / 10;
            }
        } else {
            realAngle = getAngle();
        }

		atDesiredAngle = atDesiredAngle().getAsBoolean();

		RobotContainer.components3d[NTConstants.PIVOT_INDEX] = new Pose3d(
			NTConstants.PIVOT_OFFSET_METERS.getX(),
			0,
			NTConstants.PIVOT_OFFSET_METERS.getZ(),
			new Rotation3d(0, Units.degreesToRadians(getAngle()), 0)
		);
        pivotPID.updatePID();
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
		return pivotEncoder.getPosition();
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
	public BooleanSupplier atDesiredAngle() {
		return () -> (MathUtil.applyDeadband(
				Math.abs(
						getAngle() - getTargetAngle()),
				ShooterConstants.PIVOT_DEADBAND) == 0);
	}
}