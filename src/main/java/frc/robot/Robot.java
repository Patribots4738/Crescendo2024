package frc.robot;

import java.util.Optional;

import org.littletonrobotics.urcl.URCL;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Neo;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.FieldConstants.GameMode;
import frc.robot.util.Constants.NeoMotorConstants;
import monologue.Monologue;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command autonomousCommand;

    private RobotContainer robotContainer;

    public static double currentTimestamp = 0;
    public static double previousTimestamp = 0;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        Monologue.setupMonologue(robotContainer, "Robot", false, false);

        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);
        DriverStation.silenceJoystickConnectionWarning(true);
        // URCL.start();
}
    /**
     * This function is called every 20 ms, no matter the mode. Used for items like
     * diagnostics
     * ran during disabled, autonomous, teleoperated and test. :D
     * <p>
     * This runs *after* the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        Monologue.updateAll();
        CommandScheduler.getInstance().run();

        Robot.previousTimestamp = Robot.currentTimestamp;
        Robot.currentTimestamp = Timer.getFPGATimestamp();

    }

    @Override
    public void disabledInit() {
        robotContainer.onDisabled();
        FieldConstants.GAME_MODE = GameMode.DISABLED;
    }

    @Override
    public void disabledPeriodic() {
        // Now while this may not necessarily be a constant...
        // it needs to be updated.
        DriverStation.refreshData();
        FieldConstants.ALLIANCE = DriverStation.getAlliance();
    }

    @Override
    public void disabledExit() {
        robotContainer.onEnabled();
    }

    @Override
    public void autonomousInit() {
        DriveConstants.MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;
        DriverStation.refreshData();
        FieldConstants.ALLIANCE = DriverStation.getAlliance();
        FieldConstants.GAME_MODE = GameMode.AUTONOMOUS;
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        // Stop our autonomous command if it is still running.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        FieldConstants.GAME_MODE = GameMode.TELEOP;
        DriveConstants.MAX_SPEED_METERS_PER_SECOND = DriveConstants.MAX_TELEOP_SPEED_METERS_PER_SECOND;
        robotContainer.onEnabled();

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        FieldConstants.GAME_MODE = GameMode.TEST;
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
        // Switch back to the normal button loop!
        CommandScheduler.getInstance().setActiveButtonLoop(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
        FieldConstants.ALLIANCE = DriverStation.getAlliance();

        for (Neo neo : NeoMotorConstants.MOTOR_LIST) {
            neo.tick();
        }
    }

    public static boolean isRedAlliance() {
        return FieldConstants.ALLIANCE.equals(Optional.of(Alliance.Red));
    }

    public static boolean isBlueAlliance() {
        return FieldConstants.ALLIANCE.equals(Optional.of(Alliance.Blue));
    }
}
