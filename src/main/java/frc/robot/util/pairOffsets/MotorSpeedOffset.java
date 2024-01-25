package frc.robot.util.pairOffsets;
import edu.wpi.first.math.Pair;

public class MotorSpeedOffset extends Pair<Double, Double>{
    private OffsetMotor offsetMotor = OffsetMotor.NONE;

    public MotorSpeedOffset(double speed, double offset, OffsetMotor offsetMotor) {
        super(speed, offset);
        this.offsetMotor = offsetMotor;
    }

    public double getSpeed() {
        return super.getFirst();
    }

    public double getOffset() {
        return super.getSecond();
    }

    public OffsetMotor getOffsetMotor() {
        return this.offsetMotor;
    }

    public static MotorSpeedOffset of(double speed, double offset, OffsetMotor offsetMotor) {
        return new MotorSpeedOffset(speed, offset, offsetMotor);
    }

}
