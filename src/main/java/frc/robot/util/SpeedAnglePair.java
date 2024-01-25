package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;

public class SpeedAnglePair extends Pair<MotorSpeedOffset, Double> {

    public SpeedAnglePair(MotorSpeedOffset speed, Double angle) {
        super(speed, angle);
    }

    public MotorSpeedOffset getMotorSpeedOffset() {
        return super.getFirst();
    }

    public Double getAngle() {
        return super.getSecond();
    }

    public SpeedAnglePair changeOffsetMotor(OffsetMotor offsetMotor) {
        return SpeedAnglePair.of(
            new MotorSpeedOffset(
                this.getMotorSpeedOffset().getSpeed(),
                this.getMotorSpeedOffset().getOffset(),
                offsetMotor
            ),
            this.getAngle()
        );
    }

    public static SpeedAnglePair of(MotorSpeedOffset speed, Double angle) {
        return new SpeedAnglePair(speed, angle);
    }

    /**
     * The SpeedAnglePair that is made the bottom motor-pair is the one used as the reference for what the base offset motor is.
     * Meaning that whatever the OffsetMotor of the bottom SpeedAnglePair is, the top SpeedAnglePair will have its offset motor changed to the corresponding OffsetMotor.
     * 
     * If the bottom motor dictates the offset motor to be OffsetMotor.BOTH, then the top and bottom SpeedAnglePairs will 
     * have their offset motors changed to OffsetMotor.NONE and the offsets will be applied to the top and bottom motors respectively.
     *  
     * @param top The SpeedAnglePair that is the top motor-pair
     * @return An array of two SpeedAnglePairs, the first one is the bottom motor-pair and the second one is the top motor-pair
     */
    //TODO: check if this works as intended (it should, but im tired)
    public SpeedAnglePair[] getPairOffset(SpeedAnglePair top){
        if(this.getMotorSpeedOffset().getOffsetMotor() == OffsetMotor.BOTH){
            SpeedAnglePair both = SpeedAnglePair.of(
                new MotorSpeedOffset(
                    this.getMotorSpeedOffset().getSpeed() + this.getMotorSpeedOffset().getSpeed(),
                    this.getMotorSpeedOffset().getOffset() + this.getMotorSpeedOffset().getOffset(),
                    OffsetMotor.NONE // ensure that the offset is not applied twice
                ),
                this.getAngle()
            );
            // no need to change the offset motor for both because it is already applied to the top and bottom
            return new SpeedAnglePair[]{
                both,
                both
            };
        } else if(this.getMotorSpeedOffset().getOffsetMotor() == OffsetMotor.TOP){
            SpeedAnglePair newTopPair = SpeedAnglePair.of(
                new MotorSpeedOffset(
                    top.getMotorSpeedOffset().getSpeed(),
                    top.getMotorSpeedOffset().getOffset(),
                    OffsetMotor.NONE // ensure that the offset is not applied twice
                ),
                top.getAngle()
            );
            // TODO: check if this works as intended (it should, but im tired)
            this.changeOffsetMotor(OffsetMotor.NONE); // ensure that the offset is not applied twice
            return new SpeedAnglePair[]{
                newTopPair,
                this
            };
        } else if(this.getMotorSpeedOffset().getOffsetMotor() == OffsetMotor.BOTTOM){
            SpeedAnglePair newBottomPair = SpeedAnglePair.of(
                new MotorSpeedOffset(
                    this.getMotorSpeedOffset().getSpeed(),
                    this.getMotorSpeedOffset().getOffset(),
                    OffsetMotor.NONE // ensure that the offset is not applied twice
                ),
                this.getAngle()
            );
            top = top.changeOffsetMotor(OffsetMotor.NONE); // ensure that the offset is not applied twice
            return new SpeedAnglePair[]{
                top,
                newBottomPair
            };
        }
        return new SpeedAnglePair[]{
            this,
            top,
        };
    }

    public SpeedAnglePair interpolate(SpeedAnglePair other, double t) {
        return SpeedAnglePair.of(
            new MotorSpeedOffset(
                MathUtil.interpolate(
                    this.getMotorSpeedOffset().getSpeed(), 
                    other.getMotorSpeedOffset().getSpeed(), 
                    t), 
                0, 
                OffsetMotor.NONE),
            MathUtil.interpolate(
                this.getAngle(), 
                other.getAngle(), 
                t)
        );
    }
    
}
