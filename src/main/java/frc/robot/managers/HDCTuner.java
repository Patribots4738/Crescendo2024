package frc.robot.managers;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import monologue.Annotations.Log;
import monologue.Logged;

/**
 * This class is used to control the calibration of the HDC
 * for fast and easy tuning of the HDC values.
 * 
 * The HDC is controlled by the HolonomicDriveController class
 * which contains the XY and Theta controllers.
 * 
 * The values are updated instantly and the HDC is updated.
 * After calibration, the values can be logged to the console for use in the code.
 */
public class HDCTuner extends SubsystemBase implements Logged {

    @Log
    public int PIDControllerIndex = 0, PIDIndex = 0;

    public PIDController XYController = AutoConstants.HDC.getXController();

    public ProfiledPIDController thetaController = AutoConstants.HDC.getThetaController();

    public HDCTuner(PIDController XYController, ProfiledPIDController thetaController) {
        this.XYController = XYController;
        this.thetaController = thetaController;
    }

    /**
     * This method is used to update the values of the HDC
     * after the PID values are changed.
     */
    public void updateValues() {
        AutoConstants.HDC = new HolonomicDriveController(
                XYController,
                XYController,
                thetaController);
    }

    /**
     * This method is used to log the values of the HDC
     * to the console for use in the code.
     */
    public void log() {
        updateValues();
        System.out.printf("XY P: %.4f | I: %.4f | D: %.4f | Theta P: %.4f | I: %.4f | D: %.4f%n",
                XYController.getP(), XYController.getI(), XYController.getD(),
                thetaController.getP(), thetaController.getI(), thetaController.getD());
    }

    /**
     * This method is used to increment the PID values.
     */
    public void PIDIncrement() {
        if (PIDIndex < 3) {
            this.PIDIndex++;
            System.out.println("Currently editing: " + (PIDIndex == 0 ? "P" : PIDIndex == 1 ? "D" : "I"));
        }
    }

    /**
     * This method is used to decrement the PID values.
     */
    public void PIDDecrement() {
        if (PIDIndex > 0) {
            this.PIDIndex--;
            System.out.println("Currently editing: " + (PIDIndex == 0 ? "P" : PIDIndex == 1 ? "D" : "I"));
        }
    }

    /**
     * This method is used to change the controller being edited. (increment)
     */
    public void controllerIncrement() {
        if (PIDControllerIndex < 1) {
            this.PIDControllerIndex++;
            System.out.println("Currently editing: " + (PIDControllerIndex == 0 ? "XY" : "Theta"));
        }
    }

    /**
     * This method is used to change the controller being edited. (decrement)
     */
    public void controllerDecrement() {
        if (PIDControllerIndex > 0) {
            this.PIDControllerIndex--;
            System.out.println("Currently editing: " + (PIDControllerIndex == 0 ? "XY" : "Theta"));
        }
    }

    /**
     * This method is used to increase the current PID value on the current controller.
     * 
     * @param value The amount to increase the PID value by
     */
    public void increasePID(double value) {
        if (PIDControllerIndex == 0) {
            switch (this.PIDIndex) {
                case 0:
                    XYController.setP(XYController.getP() + value);
                    break;
                case 1:
                    XYController.setD(XYController.getD() + value * 0.1);
                    break;
                case 2:
                    XYController.setI(XYController.getI() + value);
                    break;
            }
        } else {
            switch (this.PIDIndex) {
                case 0:
                    thetaController.setP(thetaController.getP() + value);
                    break;
                case 1:
                    thetaController.setD(thetaController.getD() + value);
                    break;
                case 2:
                    thetaController.setI(thetaController.getI() + value);
                    break;
            }
        }
        log();
    }

    /**
     * This method is used to multiply the current PID value on the current controller.
     * 
     * @param value The amount to multiply the PID value by
     */
    public void multiplyPID(double value) {
        if (PIDControllerIndex == 0) {
            switch (this.PIDIndex) {
                case 0:
                    XYController.setP(XYController.getP() * value);
                    break;
                case 1:
                    XYController.setD(XYController.getD() * value);
                    break;
                case 2:
                    XYController.setI(XYController.getI() * value);
                    break;
            }
        } else {
            switch (this.PIDIndex) {
                case 0:
                    thetaController.setP(thetaController.getP() * value);
                    break;
                case 1:
                    thetaController.setD(thetaController.getD() * value);
                    break;
                case 2:
                    thetaController.setI(thetaController.getI() * value);
                    break;
            }
        }
        log();
    }

    /**
     * This method is used to create a command that logs the values of the HDC
     * to the console for use in the code.
     * 
     * @return Command that logs the values of the HDC
     */
    public Command logCommand() {
        return Commands.runOnce(this::log);
    }
    
    /**
     * This method is used to create a command that multiplies the PID values of the HDC
     * 
     * @param scalar The amount to multiply the PID values by
     * @return       Command that multiplies the PID values of the HDC
     */
    public Command multiplyPIDCommand(double scalar) {
        return Commands.runOnce(() -> {
            this.multiplyPID(scalar);
        });
    }

    /**
     * This method is used to create a command that increments the PID values of the HDC
     * by a constant value.
     * 
     * @return Command that increments the PID values of the HDC
     */
    public Command constantIncrementCommand() {
        return Commands.runOnce(() -> {
            PIDIncrement();
        });
    }

    /**
     * This method is used to create a command that decrements the PID values of the HDC
     * by a constant value.
     * 
     * @return Command that decrements the PID values of the HDC
     */
    public Command constantDecrementCommand() {
        return Commands.runOnce(() -> {
            PIDDecrement();
        });
    }

    /**
     * This method is used to create a command that increments the controller being edited.
     * 
     * @return Command that increments the controller being edited
     */
    public Command controllerIncrementCommand() {
        return Commands.runOnce(() -> {
            controllerIncrement();
        });
    }

    /**
     * This method is used to create a command that decrements the controller being edited.
     * 
     * @return Command that decrements the controller being edited
     */
    public Command controllerDecrementCommand() {
        return Commands.runOnce(() -> {
            controllerDecrement();
        });
    }

    /**
     * This method is used to create a command that increases the current PID value on the current controller.
     * 
     * @param value The amount to increase the PID value by
     * @return      Command that increases the PID value
     */
    public Command increaseCurrentConstantCommand(double value) {
        return Commands.runOnce(() -> {
            this.increasePID(value);
        });
    }

    /**
     * This method is used to create a command that decreases the current PID value on the current controller.
     * 
     * @param value The amount to decrease the PID value by
     * @return      Command that decreases the PID value
     */
    public Command decreaseCurrentConstantCommand(double value) {
        return Commands.runOnce(() -> {
            this.increasePID(-value);
        });
    }

}
