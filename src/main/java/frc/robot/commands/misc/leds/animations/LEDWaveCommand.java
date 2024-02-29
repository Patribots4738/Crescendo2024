package frc.robot.commands.misc.leds.animations;

import java.util.Set;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.misc.leds.animations.LEDCommands.HSV;
import frc.robot.subsystems.misc.leds.LedStrip;

public class LEDWaveCommand {

    protected void evaluateWave(LedStrip ledStrip, double scale, double spacing, double time, int x, HSV isHue,
            double height, int hueVal) {
        switch (isHue) {
            case HUE -> evaluateHueWave(ledStrip, scale, spacing, time, x, height);
            case VALUE -> evaluateValueWave(ledStrip, scale, spacing, time, x, height, hueVal);
            case SATURATION -> evaluateSaturationWave(ledStrip, scale, spacing, time, x, height, hueVal);
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

        private Command waveHueFunction;

        protected void waveHueFunction() {
            for (int i = 0; i < ledStrip.getBuffer().getLength(); i++) {
                evaluateWave(
                    ledStrip,
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

        public Hue(double scale, double spacing, double height, double waveSpeed, double waveRange) {
            this(scale, spacing, height, waveSpeed, waveRange, Set.of());
        }

        public Hue(double scale, double spacing, double height, double waveSpeed, double waveRange,
                Set<Subsystem> requirements) {
            super(ledStrip, requirements);
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

        private final Command waveValueFunction;

        protected void waveValueFunction() {
            for (int i = 0; i < ledStrip.getBuffer().getLength(); i++) {
                evaluateWave(
                    ledStrip,
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

        public Value(double scale, double spacing, double height, double waveValueIncrement, double waveRange,
                int hueVal) {
            this(scale, spacing, height, waveValueIncrement, waveRange, hueVal, Set.of());
        }

        public Value(double scale, double spacing, double height, double waveValueIncrement, double waveRange,
                int hueVal, Set<Subsystem> requirements) {
            super(ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            this.waveValueIncrement = waveValueIncrement;
            this.waveRange = waveRange;
            this.hueVal = hueVal;
            this.waveValueFunction = Commands.run(() -> waveValueFunction(), requirements.toArray(new Subsystem[0]))
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

        private final Command waveSaturationFunction;

        protected void waveSaturationFunction() {
            for (int i = 0; i < ledStrip.getBuffer().getLength(); i++) {
                evaluateWave(
                    ledStrip,
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

        public Saturation(double scale, double spacing, double height, double waveSaturationIncrement, double waveRange,
                int hueVal) {
            this(scale, spacing, height, waveSaturationIncrement, waveRange, hueVal, Set.of());
        }

        public Saturation(double scale, double spacing, double height, double waveSaturationIncrement, double waveRange,
                int hueVal, Set<Subsystem> requirements) {
            super(ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
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
