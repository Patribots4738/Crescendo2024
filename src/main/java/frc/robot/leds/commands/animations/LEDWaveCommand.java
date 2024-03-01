package frc.robot.leds.commands.animations;

import java.util.Set;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.leds.LedStrip;
import frc.robot.leds.commands.animations.LEDCommands.HSV;

public class LEDWaveCommand {

    protected void evaluateWave(LEDCommands root, double scale, double spacing, double time, int x, HSV hsvMode,
            double height, int hueVal) {
        switch (hsvMode) {
            case HUE -> evaluateHueWave(root.ledStrip, scale, spacing, time, x, height);
            case VALUE -> evaluateValueWave(root.ledStrip, scale, spacing, time, x, height, hueVal);
            case SATURATION -> evaluateSaturationWave(root.ledStrip, scale, spacing, time, x, height, hueVal);
            default -> {
            }
        }
    }

    private void evaluateSaturationWave(LedStrip ledStrip, double scale, double spacing, double time, int x,
            double height, int hueVal) {
        double wave = Math.sin((10 * x + (spacing * time)) / spacing);
        int output = (int) ((height + scale * wave) / 1.0);
        ledStrip.setLED(x, Color.fromHSV(hueVal, output, 255));
    }

    private void evaluateValueWave(LedStrip ledStrip, double scale, double spacing, double time, int x, double height,
            int hueVal) {
        double wave = Math.sin((10 * x + (spacing * time)) / spacing);
        int output = (int) ((height + scale * wave) / 1.0);
        ledStrip.setLED(x, Color.fromHSV(hueVal, 255, output));
    }

    private void evaluateHueWave(LedStrip ledStrip, double scale, double spacing, double time, int x, double height) {
        double wave = Math.sin((10 * x + (spacing * time)) / spacing);
        int output = (int) ((height + scale * wave) / 2.0);
        ledStrip.setLED(x, Color.fromHSV(output, 255, 255));
    }

    public class Hue extends LEDCommands {
        private final double scale;
        private final double spacing;
        private final double height;
        private final double waveSpeed;
        private final double waveRange;
        private LEDCommands root;
        private Command waveHueFunction;

        protected void waveHueFunction() {
            for (int i = 0; i < ledStrip.getBuffer().getLength(); i++) {
                evaluateWave(
                    root,
                    scale,
                    spacing,
                    waveHueOffset,
                    i,
                    HSV.HUE,
                    height,
                    180);
            }
            waveHueOffset += waveSpeed;
            waveHueOffset %= waveRange;
        }

        public Hue(LEDCommands root, double scale, double spacing, double height, double waveSpeed, double waveRange) {
            this(root, scale, spacing, height, waveSpeed, waveRange, Set.of());
        }

        public Hue(LEDCommands root, double scale, double spacing, double height, double waveSpeed, double waveRange,
                Set<Subsystem> requirements) {
            super(root.ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            this.waveSpeed = waveSpeed;
            this.waveRange = waveRange;
            this.waveHueFunction = Commands.run(() -> waveHueFunction(), requirements.toArray(new Subsystem[0]))
                    .ignoringDisable(true);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            waveHueFunction.execute();
        }

    }

    public class Value extends LEDCommands {
        private final double waveValueIncrement;
        private final double waveRange;
        private final double scale;
        private final double spacing;
        private final double height;
        private final int hueVal;
        private LEDCommands root;
        private final Command waveValueFunction;

        protected void waveValueFunction() {
            for (int i = 0; i < root.ledStrip.getBuffer().getLength(); i++) {
                evaluateWave(
                    root,
                    scale,
                    spacing,
                    waveValueOffset,
                    i,
                    HSV.VALUE,
                    height,
                    hueVal);
            }
            waveValueOffset += waveValueIncrement;
            waveValueOffset %= waveRange;
        }

        public Value(LEDCommands root, double scale, double spacing, double height, double waveValueIncrement, double waveRange,
                int hueVal) {
            this(root, scale, spacing, height, waveValueIncrement, waveRange, hueVal, Set.of());
        }

        public Value(LEDCommands root, double scale, double spacing, double height, double waveValueIncrement, double waveRange,
                int hueVal, Set<Subsystem> requirements) {
            super(root.ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            this.waveValueIncrement = waveValueIncrement;
            this.waveRange = waveRange;
            this.hueVal = hueVal;
            // we wrap this in a commands.run so that we can run it without worrying about the end condition
            // as we want this to run indefinitely until the top level command is ended
            this.waveValueFunction = 
                Commands.run(() -> waveValueFunction(), 
                    requirements.toArray(new Subsystem[0]))
                    .ignoringDisable(true);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            waveValueFunction.execute();
        }
    }

    public class Saturation extends LEDCommands {
        private final double waveSaturationIncrement;
        private final double waveRange;
        private final double scale;
        private final double spacing;
        private final double height;
        private final int hueVal;
        private LEDCommands root;
        private final Command waveSaturationFunction;

        protected void waveSaturationFunction() {
            for (int i = 0; i < root.ledStrip.getBuffer().getLength(); i++) {
                evaluateWave(
                    root,
                    scale,
                    spacing,
                    waveValueOffset,
                    i,
                    HSV.SATURATION,
                    height,
                    hueVal);
            }
            waveValueOffset += waveSaturationIncrement;
            waveValueOffset %= waveRange;
        }

        public Saturation(LEDCommands root, double scale, double spacing, double height, double waveSaturationIncrement, double waveRange,
                int hueVal) {
            this(root, scale, spacing, height, waveSaturationIncrement, waveRange, hueVal, Set.of());
        }

        public Saturation(LEDCommands root, double scale, double spacing, double height, double waveSaturationIncrement, double waveRange,
                int hueVal, Set<Subsystem> requirements) {
            super(root.ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            // this is the speed of the wave
            this.waveSaturationIncrement = waveSaturationIncrement;
            this.waveRange = waveRange;
            this.hueVal = hueVal;
            this.waveSaturationFunction = Commands
                .run(() -> waveSaturationFunction(), requirements.toArray(new Subsystem[0])).ignoringDisable(true);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            waveSaturationFunction.execute();
        }
    }
}
