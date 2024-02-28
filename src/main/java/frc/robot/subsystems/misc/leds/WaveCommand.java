package frc.robot.subsystems.misc.leds;

import java.util.Set;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

class WaveCommand extends LEDFunctionCommand {
    static double waveHueOffset = 1;
    static double waveValueOffset = 1;


    public WaveCommand(LedStrip ledStrip) {
        super(ledStrip);
    }

    public WaveCommand(LedStrip ledStrip, Set<Subsystem> requirements) {
        super(ledStrip, requirements);
    }

    protected void evaluateWave(double scale, double spacing, double time, int x, boolean isHue, double height) {
        double wave = Math.sin((10 * x + (spacing * time)) / spacing);
        int output = (int) ((height + scale * wave) / (isHue ? 2 : 1));
        ledStrip.setLED(x, isHue ? Color.fromHSV(output, 255, 255) : Color.fromHSV(180, 255, output));
    }

    protected class Hue extends WaveCommand {
        private final double scale;
        private final double spacing;
        private final double height;
        private final double waveSpeed;
        private final double waveRange;


        private Command waveHueFunction;

        protected void waveHueFunction() {
            for (int i = 0; i < ledStrip.ledBuffer.getLength(); i++) {
                evaluateWave(
                    scale,
                    spacing,
                    waveHueOffset,
                    i,
                    true,
                    height);
            }
            waveHueOffset += waveSpeed;
            waveHueOffset %= waveRange;
        }

        public Hue(double scale, double spacing, double height, double waveSpeed, double waveRange) {
            this(scale, spacing, height, waveSpeed, waveRange, Set.of());
        }

        public Hue(double scale, double spacing, double height, double waveSpeed, double waveRange, Set<Subsystem> requirements) {
            super(ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            this.waveSpeed = waveSpeed;
            this.waveRange = waveRange;
            this.waveHueFunction = Commands.run(() -> waveHueFunction(), requirements.toArray(new Subsystem[0])).ignoringDisable(true);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            waveHueFunction.execute();
        }

    }

    protected class Value extends WaveCommand {
        private final double waveValueIncrement;
        private final double waveRange;
        private final double scale;
        private final double spacing;
        private final double height;

        private final Command waveValueFunction;

        protected void waveValueFunction() {
            for (int i = 0; i < ledStrip.ledBuffer.getLength(); i++) {
                evaluateWave(
                        scale,
                        spacing,
                        waveValueOffset,
                        i,
                        false,
                        height);
            }
            waveValueOffset += waveValueIncrement;
            waveValueOffset %= waveRange;
        }

        public Value(LedStrip ledStrip, double scale, double spacing, double height, double waveValueIncrement, double waveRange) {
            this(ledStrip, scale, spacing, height, waveValueIncrement, waveRange, Set.of());
        }

        public Value(LedStrip ledStrip, double scale, double spacing, double height, double waveValueIncrement, double waveRange, Set<Subsystem> requirements) {
            super(ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            this.waveValueIncrement = waveValueIncrement;
            this.waveRange = waveRange;
            this.waveValueFunction = Commands.runOnce(() -> waveValueFunction(), requirements.toArray(new Subsystem[0])).ignoringDisable(true);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            waveValueFunction.initialize();
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            waveValueFunction.execute();
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            waveValueFunction.end(interrupted);
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return waveValueFunction.isFinished();
        }
    }

}
