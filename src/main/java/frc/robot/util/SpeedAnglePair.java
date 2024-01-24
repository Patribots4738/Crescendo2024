package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;

public class SpeedAnglePair extends Pair<Double, Double> {

    public SpeedAnglePair(Double speed, Double angle) {
        super(speed, angle);
    }

    public Double getSpeed() {
        return super.getFirst();
    }

    public Double getAngle() {
        return super.getSecond();
    }

    public static SpeedAnglePair of(Double speed, Double angle) {
        return new SpeedAnglePair(speed, angle);
    }

    public SpeedAnglePair interpolate(SpeedAnglePair other, double t) {
        return SpeedAnglePair.of(
            MathUtil.interpolate(this.getSpeed(), other.getSpeed(), t),
            MathUtil.interpolate(this.getAngle(), other.getAngle(), t)
        );
    }
    
}
