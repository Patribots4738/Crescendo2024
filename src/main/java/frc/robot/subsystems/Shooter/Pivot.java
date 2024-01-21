package frc.robot.subsystems.Shooter;

public class Pivot {
    pivot = new Neo(ShooterConstants.SHOOTER_PIVOT_CAN_ID);
    configMotor();
}

public void configMotor() {
        pivot.setSmartCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT);
        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
        pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        pivot.setInverted(false);

        Pivot.setPID(
                ShooterConstants.SHOOTER_P,
                ShooterConstants.SHOOTER_I,
                ShooterConstants.SHOOTER_D,
                ShooterConstants.SHOOTER_MIN_OUTPUT,
                ShooterConstants.SHOOTER_MAX_OUTPUT);


        //sets brake mode
        pivot.setBrakeMode();
    }
    
public void setAngle(double angle) {
    pivot.setTargetPosition(angle);
}