package frc.robot.util;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class HDCTuner extends SubsystemBase implements Logged {

    @Log
    public double XYkP;
    @Log
    public double XYkI;
    @Log
    public double XYkD;
    @Log
    public double thetaKp;
    @Log
    public double thetaKi;
    @Log
    public double thetaKd;

    @Log
    public double thetaMaxV;
    @Log
    public double thetaMaxA;

    @Log
    public int HIDControllerIndex = 0;

    public TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(0,0);

    public PIDController XYController = new PIDController(0,0,0);

    public ProfiledPIDController thetaController = 
            new ProfiledPIDController( 0, 0, 0,
            new TrapezoidProfile.Constraints(0,0));

    public HolonomicDriveController HDC = new HolonomicDriveController(
        XYController,
        XYController,
        thetaController
    );

    public HDCTuner(double XYkP, double XYkI, double XYkD, double thetaKp, double thetaKi, double thetaKd, double thetaMaxV, double thetaMaxA) {
        this.XYkP = XYkP;
        this.XYkI = XYkI;
        this.XYkD = XYkD;
        this.thetaKp = thetaKp;
        this.thetaKi = thetaKi;
        this.thetaKd = thetaKd;
        this.thetaConstraints = new TrapezoidProfile.Constraints(thetaMaxV, thetaMaxA);
        this.XYController = new PIDController(XYkP, XYkI, XYkD);
        this.thetaController = new ProfiledPIDController(thetaKp, thetaKi, thetaKd, thetaConstraints);
        this.HDC = new HolonomicDriveController(
            XYController,
            XYController,
            thetaController
        );
        this.HIDControllerIndex = 0;
    }

    public HDCTuner(PIDController XYController, ProfiledPIDController thetaController) {
        this(XYController.getP(), XYController.getI(), XYController.getD(), thetaController.getP(), thetaController.getI(), thetaController.getD(), thetaController.getConstraints().maxVelocity, thetaController.getConstraints().maxAcceleration);
    }

    public HDCTuner(double XYkP, double XYkI, double XYkD, ProfiledPIDController thetaController) {
        this(XYkP, XYkI, XYkD, thetaController.getP(), thetaController.getI(), thetaController.getD(), thetaController.getConstraints().maxVelocity, thetaController.getConstraints().maxAcceleration);
    }

    public HDCTuner(PIDController XYController, double thetaKp, double thetaKi, double thetaKd, TrapezoidProfile.Constraints thetaConstraints) {
        this(XYController.getP(), XYController.getI(), XYController.getD(), thetaKp, thetaKi, thetaKd, thetaConstraints.maxVelocity, thetaConstraints.maxAcceleration);
    }

    @Override
    public void periodic() {
        this.HDC = new HolonomicDriveController(
            XYController,
            XYController,
            thetaController
        );
    }

    @Override
    public String toString() {
        return String.format("XY P: %.6f | I: %.6f | D: %.6f | Theta P: %.6f | I: %.6f | D: %.6f", this.XYkP, this.XYkI, this.XYkD, this.thetaKp, this.thetaKi, this.thetaKd);
    }

    public PatrIDConstants getXYPID() {
        return new PatrIDConstants(this.XYkP, this.XYkI, this.XYkD);
    }

    public void setXYP(double value) {
        XYkP = value;
    }

    public void setXYI(double value) {
        XYkI = value;
    }

    public void setXYD(double value) {
        XYkD = value;
    }

    public double getXYP() {
        return XYkP;
    }

    public double getXYI() {
        return XYkI;
    }

    public double getXYD() {
        return XYkD;
    }
    
    public void setHDC(HolonomicDriveController HDC) {
        this.HDC = HDC;
    }

    public void setXYController(PIDController XYController) {
        this.XYController = XYController;
    }

    public void setXYController(double kP, double kI, double kD) {
        this.XYController = new PIDController(kP, kI, kD);
    }

    public void setThetaController(ProfiledPIDController thetaController) {
        this.thetaController = thetaController;
    }

    public void setThetaController(double kP, double kI, double kD, double ff) {
        this.thetaController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(0, ff));
    }

    public void setHDC(PIDController XYController, ProfiledPIDController thetaController) {
        this.HDC = new HolonomicDriveController(
            XYController,
            XYController,
            thetaController
        );
    }

    public HolonomicDriveController getHDC() {
        return this.HDC;
    }

    public PIDController getXYController() {
        return this.XYController;
    }

    public ProfiledPIDController getThetaController() {
        return this.thetaController;
    }

    public Command logCommand() {
        return Commands.runOnce(() -> {
            System.out.println(this.toString());
        });
    }

    public Command incrementHIDControllerIndexCommand() {
        return Commands.runOnce(() -> {
            this.HIDControllerIndex++;
            if (this.HIDControllerIndex > 3) {
                this.HIDControllerIndex = 0;
            }
            System.out.println("HIDControllerIndex: " + this.HIDControllerIndex);
        });
    }

    public Command decreaseHIDControllerIndexCommand() {
        return Commands.runOnce(() -> {
            this.HIDControllerIndex--;
            if (this.HIDControllerIndex < 0) {
                this.HIDControllerIndex = 3;
            }
            System.out.println("HIDControllerIndex: " + this.HIDControllerIndex);
        });
    }
    
    public Command multiplyPIDCommand(double d) {
        return Commands.sequence(
            multiplyPCommand(d),
            multiplyICommand(d),
            multiplyDCommand(d)
        );
    }

    public Command multiplyPCommand(double d) {
        return Commands.runOnce(() -> {
            switch (this.HIDControllerIndex) {
                case 0:
                    this.XYkP *= d;
                    break;
                case 1:
                    this.thetaKp *= d;
                    break;
                case 2:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity, this.thetaConstraints.maxAcceleration * d);
                    break;
                case 3:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity * d, this.thetaConstraints.maxAcceleration);
                    break;
            }
            System.out.println(this.toString());
        });
    }

    public Command multiplyICommand(double d) {
        return Commands.runOnce(() -> {
            switch (this.HIDControllerIndex) {
                case 0:
                    this.XYkI *= d;
                    break;
                case 1:
                    this.thetaKi *= d;
                    break;
                case 2:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity, this.thetaConstraints.maxAcceleration * d);
                    break;
                case 3:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity * d, this.thetaConstraints.maxAcceleration);
                    break;
            }
            System.out.println(this.toString());
        });
    }

    public Command multiplyDCommand(double d) {
        return Commands.runOnce(() -> {
            switch (this.HIDControllerIndex) {
                case 0:
                    this.XYkD *= d;
                    break;
                case 1:
                    this.thetaKd *= d;
                    break;
                case 2:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity, this.thetaConstraints.maxAcceleration * d);
                    break;
                case 3:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity * d, this.thetaConstraints.maxAcceleration);
                    break;
            }
            System.out.println(this.toString());
        });
    }

    public Command incrementPIDCommand(double d) {
        return Commands.sequence(
            incrementPCommand(d),
            incrementICommand(d),
            incrementDCommand(d)
        );
    }

    public Command incrementPIDCommand() {
        return Commands.sequence(
            incrementPCommand(),
            incrementICommand(),
            incrementDCommand()
        );
    }

    public Command decreasePIDCommand() {
        return Commands.sequence(
            decreasePCommand(),
            decreaseICommand(),
            decreaseDCommand()
        );
    }

    public Command incrementPCommand(double d) {
        return Commands.runOnce(() -> {
            switch (this.HIDControllerIndex) {
                case 0:
                    this.XYkP += d;
                    break;
                case 1:
                    this.thetaKp += d;
                    break;
                case 2:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity, this.thetaConstraints.maxAcceleration + d);
                    break;
                case 3:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity + d, this.thetaConstraints.maxAcceleration);
                    break;
            }
            System.out.println(this.toString());
        });
    }

    public Command incrementICommand(double d) {
        return Commands.runOnce(() -> {
            switch (this.HIDControllerIndex) {
                case 0:
                    this.XYkI += d;
                    break;
                case 1:
                    this.thetaKi += d;
                    break;
                case 2:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity, this.thetaConstraints.maxAcceleration + d);
                    break;
                case 3:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity + d, this.thetaConstraints.maxAcceleration);
                    break;
            }
            System.out.println(this.toString());
        });
    }

    public Command incrementDCommand(double d) {
        return Commands.runOnce(() -> {
            switch (this.HIDControllerIndex) {
                case 0:
                    this.XYkD += d;
                    break;
                case 1:
                    this.thetaKd += d;
                    break;
                case 2:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity, this.thetaConstraints.maxAcceleration + d);
                    break;
                case 3:
                    this.thetaConstraints = new TrapezoidProfile.Constraints( this.thetaConstraints.maxVelocity + d, this.thetaConstraints.maxAcceleration);
                    break;

            }
            System.out.println(this.toString());
        });
    }

    public Command incrementPCommand() {
        return incrementPCommand(.01);
    }

    public Command decreasePCommand() {
        return incrementPCommand(-.01);
    }

    public Command incrementICommand() {
        return incrementICommand(.01);
    }

    public Command decreaseICommand() {
        return incrementICommand(-.01);
    }

    public Command incrementDCommand() {
        return incrementDCommand(.01);
    }

    public Command decreaseDCommand() {
        return incrementDCommand(-.01);
    }

}
