package frc.robot.util.mod;

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
    public SpeedAngleTriplet(Double leftSpeed, Double rightSpeed, Double angle) {
        super(new Pair<Double, Double>(leftSpeed, rightSpeed), angle);
    }

    public SpeedAngleTriplet(Pair<Double, Double> speeds, Double angle) {
        super(speeds, angle);
    }

    public SpeedAngleTriplet() {
        super(new Pair<Double, Double>(0.0, 0.0), 0.0);
    }

    /**
     * Returns the speed value for the left motor
     *
     * @return the speed value, in RPM
     */
    public Double getLeftSpeed() {
        return super.getFirst().getFirst();
    }

    /**
     * Returns the speed value for the left motor
     *
     * @return the speed value, in RPM
     */
    public Double getRightSpeed() {
        return super.getFirst().getSecond();
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
    public Double getAngle() {
        return super.getSecond();
    }

    public static SpeedAngleTriplet of(Double leftSpeed, Double rightSpeed, Double angle) {
        return new SpeedAngleTriplet(leftSpeed, rightSpeed, angle);
    }

    public static SpeedAngleTriplet of(Pair<Double, Double> speeds, Double angle) {
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
