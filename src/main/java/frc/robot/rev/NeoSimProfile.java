package frc.robot.rev;

import edu.wpi.first.math.system.plant.DCMotor;

class NeoSimProfile extends NeoPhysicsSim.SimProfile {
    private final Neo neo;
    private double stallTorque = 0.0;
    private double freeSpeed = 0.0;
    private DCMotor DCMotor = null;
    private double velocity = 0.0;
    private double RPM_CONVERSION = 9.5493;

   public NeoSimProfile(Neo neo, double stallTorque, double freeSpeed) {
      this.neo = neo;
      this.stallTorque = stallTorque;
      this.freeSpeed = freeSpeed;
      neo.setSimFreeSpeed((float)this.freeSpeed);
      neo.setSimStallTorque((float)this.stallTorque);
   }

   public NeoSimProfile(Neo neo, DCMotor motor) {
      this.neo = neo;
      this.DCMotor = motor;
      neo.setSimFreeSpeed((float)(this.DCMotor.freeSpeedRadPerSec * this.RPM_CONVERSION));
      neo.setSimStallTorque((float)this.DCMotor.stallTorqueNewtonMeters);
   }

    public void run() {
        double period = this.getPeriod();
        this.velocity = this.neo.getEncoder().getVelocity();
        double position = this.neo.getEncoder().getPosition();
        double posFactor = this.neo.getEncoder().getPositionConversionFactor();
        this.neo.getEncoder().setPosition(position + this.velocity * period / 60000.0 * posFactor);
    }
}
