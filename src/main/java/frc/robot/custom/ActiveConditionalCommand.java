package frc.robot.custom;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.BooleanSupplier;

public class ActiveConditionalCommand extends Command {
    
    private final Command onTrueCommand;
    private final Command onFalseCommand;
    private final BooleanSupplier condition;
    private Command selectedCommand;

    public ActiveConditionalCommand(Command onTrueCommand, Command onFalseCommand, BooleanSupplier condition) {
        this.onTrueCommand = requireNonNullParam(onTrueCommand, "onTrue", "ConditionalCommand");
        this.onFalseCommand = requireNonNullParam(onFalseCommand, "onFalse", "ConditionalCommand");
        this.condition = requireNonNullParam(condition, "condition", "ConditionalCommand");

        CommandScheduler.getInstance().registerComposedCommands(onTrueCommand, onFalseCommand);

        m_requirements.addAll(this.onTrueCommand.getRequirements());
        m_requirements.addAll(this.onFalseCommand.getRequirements());
    }

    @Override
    public void initialize() {
        if (condition.getAsBoolean()) {
            selectedCommand = onTrueCommand;
        } else {
            selectedCommand = onFalseCommand;
        }
        selectedCommand.initialize();
    }

    @Override
    public void execute() {
        if (condition.getAsBoolean() ^ (selectedCommand == onTrueCommand))
        {
            selectedCommand.end(true);
            this.initialize();
        }
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
