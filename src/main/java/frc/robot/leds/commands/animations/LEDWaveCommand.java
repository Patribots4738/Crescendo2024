package frc.robot.leds.commands.animations;

import java.util.Set;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.leds.LedStrip.PatriColors;
import frc.robot.leds.commands.animations.LEDCommands.HSV_Flash;

public class LEDWaveCommand {

    private void evaluateWave(LEDCommands root, double scale, double spacing, double time, int x, HSV_Flash hsvMode,
            double height, int hueVal) {
        
        if (root == null)
            return;
        
        double wave = Math.sin((10 * x + (spacing * time)) / spacing);
        int output = (int) ((height + scale * wave));
        
        switch (hsvMode) {
            case HUE -> root.ledStrip.setLED(x, PatriColors.fromHSV360(output, 255, 255));
            case VALUE -> root.ledStrip.setLED(x, PatriColors.fromHSV360(hueVal, 255, output));
            case SATURATION -> root.ledStrip.setLED(x, PatriColors.fromHSV360(hueVal, output, 255));
            case FLASH -> {
                output = (int) Math.signum(wave);
                if(output > 0) {
                    root.ledStrip.setLED(PatriColors.fromHSV360(hueVal, 255, 255));
                } else {
                    root.ledStrip.setLED(PatriColors.fromHSV360(0, 0, 0));
                }
              }
            default -> {
            }
        }
    }

    public class Hue extends LEDCommands {
        private final double scale;
        private final double spacing;
        private final double height;
        private final double waveSpeed;
        private final double waveRange;
        private final int index1;
        private final int index2;
        private LEDCommands root;
        private Command waveHueFunction;

        private void waveHueFunction() {
            for (int i = index1; i < index2; i++) {
                evaluateWave(
                    root,
                    scale,
                    spacing,
                    hueTime,
                    i,
                    HSV_Flash.HUE,
                    height,
                    180);
            }
            hueTime += waveSpeed;
            hueTime %= waveRange;
        }

        public Hue(LEDCommands root, double scale, double spacing, double height, double waveSpeed, double waveRange) {
            this(root, scale, spacing, height, waveSpeed, waveRange, -1, -1);
        }

        private Hue(LEDCommands root, double scale, double spacing, double height, double waveSpeed,double waveRange, int index1, int index2) {
            this(root, scale, spacing, height, waveSpeed, waveRange, index1, index2, Set.of());
        }

        public Hue(LEDCommands root, double scale, double spacing, double height, double waveSpeed, double waveRange, int index1, int index2,
                Set<Subsystem> requirements) {
            super(root.ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            this.waveSpeed = waveSpeed;
            this.waveRange = waveRange;
            if (index1 == -1 && index2 == -1) {
                this.index1 = 0;
                this.index2 = ledStrip.getBuffer().getLength();
            } else {
                this.index1 = index1;
                this.index2 = index2;
            }
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
        private final int index1;
        private final int index2;
        private LEDCommands root;
        private final Command waveValueFunction;

        private void waveValueFunction() {
            for (int i = index1; i < index2; i++) {
                evaluateWave(
                    root,
                    scale,
                    spacing,
                    valueTime,
                    i,
                    HSV_Flash.VALUE,
                    height,
                    hueVal);
            }
            valueTime += waveValueIncrement;
            valueTime %= waveRange;
        }

        public Value(LEDCommands root, double scale, double spacing, double height, double waveValueIncrement, double waveRange,
                int hueVal) {
            this(root, scale, spacing, height, waveValueIncrement, waveRange, hueVal, -1, -1);
        }

        public Value(LEDCommands root, double scale, double spacing, double height, double waveValueIncrement, double waveRange, int hueVal, int index1, int index2) {
            this(root, scale, spacing, height, waveValueIncrement, waveRange, hueVal, index1, index2, Set.of());
        }

        public Value(LEDCommands root, double scale, double spacing, double height, double waveValueIncrement, double waveRange,
                int hueVal, int index1, int index2, Set<Subsystem> requirements) {
            super(root.ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            this.waveValueIncrement = waveValueIncrement;
            this.waveRange = waveRange;
            this.hueVal = hueVal;
            if (index1 == -1 && index2 == -1) {
                this.index1 = 0;
                this.index2 = ledStrip.getBuffer().getLength();
            } else {
                this.index1 = index1;
                this.index2 = index2;
            }
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
        private final int index1;
        private final int index2;
        private LEDCommands root;
        private final Command waveSaturationFunction;

        private void waveSaturationFunction() {
            for (int i = index1; i < index2; i++) {
                evaluateWave(
                    root,
                    scale,
                    spacing,
                    valueTime,
                    i,
                    HSV_Flash.SATURATION,
                    height,
                    hueVal);
            }
            valueTime += waveSaturationIncrement;
            valueTime %= waveRange;
        }

        public Saturation(LEDCommands root, double scale, double spacing, double height, double waveSaturationIncrement, double waveRange,
                int hueVal) {
            this(root, scale, spacing, height, waveSaturationIncrement, waveRange, hueVal, -1, -1);
        }

        public Saturation(LEDCommands root, double scale, double spacing, double height, double waveSaturationIncrement, double waveRange, int hueVal, int index1, int index2) {
            this(root, scale, spacing, height, waveSaturationIncrement, waveRange, hueVal, index1, index2, Set.of());
        }

        public Saturation(LEDCommands root, double scale, double spacing, double height, double waveSaturationIncrement, double waveRange,
                int hueVal,int index1, int index2, Set<Subsystem> requirements) {
            super(root.ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            // this is the speed of the wave
            this.waveSaturationIncrement = waveSaturationIncrement;
            this.waveRange = waveRange;
            this.hueVal = hueVal;
            if (index1 == -1 && index2 == -1) {
                this.index1 = 0;
                this.index2 = ledStrip.getBuffer().getLength();
            } else {
                this.index1 = index1;
                this.index2 = index2;
            }
            this.waveSaturationFunction = Commands
                .run(() -> waveSaturationFunction(), requirements.toArray(new Subsystem[0])).ignoringDisable(true);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            waveSaturationFunction.execute();
        }
    }

    public class Flash extends LEDCommands {
        private LEDCommands root;
        private final double timeIncrement;
        private final double waveRange;
        private final double scale;
        private final double spacing;
        private final double height;
        private final int hueVal;
        private final double duration;
        private double startTime;
        private final int index1;
        private final int index2;

        public Flash(LEDCommands root, double scale, double spacing, double height, double timeIncrement, double waveRange,
                int hueVal, double duration) {
            this(root, scale, spacing, height, timeIncrement, waveRange, hueVal, duration, -1, -1);
        }

        public Flash(LEDCommands root, double scale, double spacing, double height, double timeIncrement, double waveRange,
                int hueVal, double duration, int index1, int index2) {
            this(root, scale, spacing, height, timeIncrement, waveRange, hueVal, duration, index1, index2, Set.of());
        }

        public Flash(LEDCommands root, double scale, double spacing, double height, double timeIncrement, double waveRange,
                int hueVal, double duration, int index1, int index2, Set<Subsystem> requirements) {
            super(root.ledStrip, requirements);
            this.scale = scale;
            this.spacing = spacing;
            this.height = height;
            this.timeIncrement = timeIncrement;
            this.waveRange = waveRange;
            this.hueVal = hueVal;
            this.root = root;
            this.duration = duration;
            if (index1 == -1 && index2 == -1) {
                this.index1 = 0;
                this.index2 = ledStrip.getBuffer().getLength();
            } else {
                this.index1 = index1;
                this.index2 = index2;
            }
        }

        @Override
        public void initialize() {
            startTime = Robot.currentTimestamp;
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            for (int i = index1; i < index2; i++) {
                evaluateWave(
                    root,
                    scale,
                    spacing,
                    valueTime,
                    i,
                    HSV_Flash.FLASH,
                    height,
                    hueVal);
            }
            valueTime += timeIncrement;
            valueTime %= waveRange;
        }

        @Override
        public boolean isFinished() {
            return Robot.currentTimestamp - startTime > duration;
        }

        @Override
        public void end(boolean interrupted) {
            root.ledStrip.setLED(Color.kBlack);
        };
    }
}
