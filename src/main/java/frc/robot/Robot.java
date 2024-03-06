package frc.robot;

import java.util.Optional;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.rev.Neo;
import frc.robot.util.rev.NeoPhysicsSim;
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

    private static Optional<Alliance> alliance = Optional.empty();
    public static GameMode gameMode = GameMode.DISABLED;
    public static enum GameMode {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST
    };

    public static double currentTimestamp = 0;
    public static double previousTimestamp = 0;

    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        Monologue.setupMonologue(robotContainer, "Robot/Draggables", false, true);

        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);
        DriverStation.silenceJoystickConnectionWarning(true);
        URCL.start(NeoMotorConstants.CAN_ID_MAP);
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
        // Set the previous to the current timestamp before it updates
        Robot.previousTimestamp = Robot.currentTimestamp;
        Robot.currentTimestamp = Timer.getFPGATimestamp();
        Monologue.updateAll();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        Robot.gameMode = GameMode.DISABLED;
        robotContainer.onDisabled();
    }
    
    @Override
    public void disabledPeriodic() {
        // Now while this may not necessarily be a constant...
        // it needs to be updated.
        DriverStation.refreshData();
        Robot.alliance = DriverStation.getAlliance();
    }

    @Override
    public void disabledExit() {
        // Shut off NetworkTables broadcasting for most logging calls
        // if we are at competition
        RobotContainer.gameModeStart = currentTimestamp;
        Monologue.setFileOnly(DriverStation.isFMSAttached());
    }

    @Override   
    public void autonomousInit() {
        // Update "constants"
        DriveConstants.MAX_SPEED_METERS_PER_SECOND = AutoConstants.MAX_SPEED_METERS_PER_SECOND;
        Robot.gameMode = GameMode.AUTONOMOUS;
        robotContainer.onEnabled();
        // We only need to update alliance becuase
        // sim GUI starts the bot in a "disconnected"
        // state which won't update the alliance before
        // we enable...
        DriverStation.refreshData();
        Robot.alliance = DriverStation.getAlliance();

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
        System.out.printf(
            "*** Auto finished in %.2f secs ***%n", Robot.currentTimestamp - RobotContainer.gameModeStart);
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        Robot.gameMode = GameMode.TELEOP;
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
        Robot.gameMode = GameMode.TEST;
        CommandScheduler.getInstance().cancelAll();
        robotContainer.onEnabled();
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
        NeoPhysicsSim.getInstance().run();
        Robot.alliance = DriverStation.getAlliance();

        for (Neo neo : NeoMotorConstants.MOTOR_MAP.values()) {
            neo.tick();
        }
    }

    public static boolean isRedAlliance() {
        return alliance.equals(Optional.of(Alliance.Red));
    }

    public static boolean isBlueAlliance() {
        return alliance.equals(Optional.of(Alliance.Blue));
    }
}
