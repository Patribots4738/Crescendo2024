package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * Represents a pair of speed and angle values.
 * Extends the Pair class to store the speed and angle as a pair.
 */
public class SpeedAnglePair extends Pair<Double, Double> {
    
    /**
     * Constructor that initializes the speed and angle with the given values.
     *
     * @param speed the speed value
     * @param angle the angle value
     */
    public SpeedAnglePair(Double speed, Double angle) {
        super(speed, angle);
    }

    /**
     * Returns the speed value.
     *
     * @return the speed value
     */
    public Double getSpeed() {
        return super.getFirst();
    }

    /**
     * Returns the angle value.
     *
     * @return the angle value
     */
    public Double getAngle() {
        return super.getSecond();
    }

    public static SpeedAnglePair of(Double speed, Double angle) {
        return new SpeedAnglePair(speed, angle);
    }

    /**
     * Interpolates between this SpeedAnglePair and another SpeedAnglePair based on a given parameter.
     * This class serves as the lower bound for the interpolation.
     *
     * @param other the other SpeedAnglePair to interpolate with
     * @param t the interpolation parameter (between 0.0 and 1.0)
     * @return the interpolated SpeedAnglePair
     */
    public SpeedAnglePair interpolate(SpeedAnglePair other, double t) {
        return SpeedAnglePair.of(
            MathUtil.interpolate(this.getSpeed(), other.getSpeed(), t),
            MathUtil.interpolate(this.getAngle(), other.getAngle(), t)
        );
    }

    /**
     * Returns an interpolator for SpeedAnglePair objects.
     *
     * @return an interpolator for SpeedAnglePair objects
     */
    public static Interpolator<SpeedAnglePair> getInterpolator() {
        return new Interpolator<SpeedAnglePair>() {
            /**
             * Interpolates between two SpeedAnglePair objects based on a given parameter.
             *
             * @param lower the lower SpeedAnglePair
             * @param upper the upper SpeedAnglePair
             * @param t the interpolation parameter (between 0.0 and 1.0)
             * @return the interpolated SpeedAnglePair
             */
            @Override
            public SpeedAnglePair interpolate(SpeedAnglePair lower, SpeedAnglePair upper, double t) {
                return lower.interpolate(upper, t);
            }
        };
    }
}
