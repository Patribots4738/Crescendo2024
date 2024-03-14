package frc.robot.subsystems;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Constants.ColorSensorConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class ColorSensor extends SubsystemBase implements Logged {
    
    private ColorSensorV3 colorSensor;

    private Trigger hasNoteTrigger;

    @Log
    private boolean hasOrange = false;
    
    private Color detectedColor = new Color(0.0, 0.0, 0.0);

    @Log
    private String colorString = "";

    private ColorMatchResult match = ColorSensorConstants.COLOR_MATCH.matchClosestColor(detectedColor);

    @Log 
    private String matchString = "";

    public ColorSensor(I2C.Port i2cPort) {
        this.colorSensor = new ColorSensorV3(i2cPort);
    }

    @Override
    public void periodic() {
        this.detectedColor = colorSensor.getColor();
        this.match = ColorSensorConstants.COLOR_MATCH.matchClosestColor(detectedColor);

        colorString = detectedColor.toString();
        if (this.match.color == ColorSensorConstants.NOTE_ORANGE) {
            matchString = "Note";
            hasOrange = true;
        } else {
            matchString = "Surely not a note";
            hasOrange = false;
        }
    }

    public boolean hasNote() {
        return this.hasOrange;
    }
}
