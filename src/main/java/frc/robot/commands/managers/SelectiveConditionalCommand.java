package frc.robot.commands.managers;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.BooleanSupplier;

/**
 * A command composition that runs a list of commands in sequence.
 *
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be
 * added to any other composition or scheduled individually, and the composition
 * requires all
 * subsystems its components require.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class SelectiveConditionalCommand extends Command {
    private final Command onTrueCommand;
    private final Command onFalseCommand;
    private final BooleanSupplier conditionSupplier;
    private Command selectedCommand;

    /**
     * Creates a new ConditionalCommand.
     *
     * @param onTrue    the command to run if the condition is true
     * @param onFalse   the command to run if the condition is false
     * @param condition the condition to determine which command to run
     */
    public SelectiveConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        onTrueCommand = requireNonNullParam(onTrue, "onTrue", "ConditionalCommand");
        onFalseCommand = requireNonNullParam(onFalse, "onFalse", "ConditionalCommand");
        conditionSupplier = requireNonNullParam(condition, "condition", "ConditionalCommand");

        CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);
    }

    @Override
    public void schedule() {
        // Only require the requirements of the command which is about to run
        m_requirements = conditionSupplier.getAsBoolean() ? onTrueCommand.getRequirements() : onFalseCommand.getRequirements();
        super.schedule();
    }

    @Override
    public void initialize() {
        if (conditionSupplier.getAsBoolean()) {
            selectedCommand = onTrueCommand;
        } else {
            selectedCommand = onFalseCommand;
        }
        selectedCommand.initialize();
    }

    @Override
    public void execute() {
        selectedCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return selectedCommand.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return onTrueCommand.runsWhenDisabled() && onFalseCommand.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (onTrueCommand.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf
                || onFalseCommand.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
            return InterruptionBehavior.kCancelSelf;
        } else {
            return InterruptionBehavior.kCancelIncoming;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("onTrue", onTrueCommand::getName, null);
        builder.addStringProperty("onFalse", onFalseCommand::getName, null);
        builder.addStringProperty(
                "selected",
                () -> {
                    if (selectedCommand == null) {
                        return "null";
                    } else {
                        return selectedCommand.getName();
                    }
                },
                null);
    }
}