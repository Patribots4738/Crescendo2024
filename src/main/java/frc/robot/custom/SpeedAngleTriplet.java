package frc.robot.custom;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * Represents a pair of speed and angle values.
 * Extends the Pair class to store the speed and angle as a pair.
 */
public class SpeedAngleTriplet extends Pair<Pair<Double, Double>, Double> implements Interpolator<SpeedAngleTriplet> {

    /**
     * Constructor that initializes the speed and angle with the given values.
     *
     * @param speed the speed value
     * @param angle the angle value
     */
    public SpeedAngleTriplet(Number leftSpeed, Number rightSpeed, Number angle) {
        super(new Pair<>(leftSpeed.doubleValue(), rightSpeed.doubleValue()), angle.doubleValue());
    }

    public SpeedAngleTriplet(Pair<Number, Number> speeds, Number angle) {
        super(new Pair<>(speeds.getFirst().doubleValue(), speeds.getSecond().doubleValue()), angle.doubleValue());
    }

    public SpeedAngleTriplet(Pair<Double, Double> speeds, Double angle) {
        super(speeds, angle);
    }

    public SpeedAngleTriplet(Double leftSpeed, Double rightSpeed, Double angle) {
        super(new Pair<>(leftSpeed, rightSpeed), angle);
    }

    public SpeedAngleTriplet() {
        super(new Pair<>(0.0, 0.0), 0.0);
    }

    /**
     * Returns the speed value for the left motor
     *
     * @return the speed value, in RPM
     */
    public double getLeftSpeed() {
        return super.getFirst().getFirst().doubleValue();
    }

    /**
     * Returns the speed value for the left motor
     *
     * @return the speed value, in RPM
     */
    public double getRightSpeed() {
        return super.getFirst().getSecond().doubleValue();
    }

    public Pair<Double, Double> getSpeeds() {
        return super.getFirst();
    }

    public double getAverageSpeed() {
        return (getLeftSpeed() + getRightSpeed()) / 2.0;
    }

    /**
     * Returns the angle value.
     *
     * @return the angle value
     */
    public double getAngle() {
        return super.getSecond().doubleValue();
    }

    public static SpeedAngleTriplet of(Number leftSpeed, Number rightSpeed, Number angle) {
        return new SpeedAngleTriplet(leftSpeed, rightSpeed, angle);
    }

    public static SpeedAngleTriplet of(Pair<Number, Number> speeds, Number angle) {
        return new SpeedAngleTriplet(speeds, angle);
    }

    /**
     * Interpolates between this SpeedAngleTriplet and another SpeedAngleTriplet
     * based on a given parameter.
     * This class serves as the lower bound for the interpolation.
     *
     * @param other the other SpeedAngleTriplet to interpolate with
     * @param t     the interpolation parameter (between 0.0 and 1.0)
     * @return the interpolated SpeedAngleTriplet
     */
    public SpeedAngleTriplet interpolate(SpeedAngleTriplet other, double t) {
        return SpeedAngleTriplet.of(
                MathUtil.interpolate(this.getLeftSpeed(), other.getLeftSpeed(), t),
                MathUtil.interpolate(this.getRightSpeed(), other.getRightSpeed(), t),
                MathUtil.interpolate(this.getAngle(), other.getAngle(), t));
    }

    /**
     * Interpolates between two SpeedAngleTriplet objects based on a given
     * parameter.
     *
     * @param lower the lower SpeedAngleTriplet
     * @param upper the upper SpeedAngleTriplet
     * @param t     the interpolation parameter (between 0.0 and 1.0)
     * @return the interpolated SpeedAngleTriplet
     */
    @Override
    public SpeedAngleTriplet interpolate(SpeedAngleTriplet lower, SpeedAngleTriplet upper, double t) {
        return lower.interpolate(upper, t);
    }

    /**
     * Returns an interpolator for SpeedAngleTriplet objects.
     *
     * @return an interpolator for SpeedAngleTriplet objects
     */
    public static Interpolator<SpeedAngleTriplet> getInterpolator() {
        return new SpeedAngleTriplet();
    }
}