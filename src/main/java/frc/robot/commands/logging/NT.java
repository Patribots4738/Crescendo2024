package frc.robot.commands.logging;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NT {
    
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("A-TimerCalibration");

    public NT(Map<String, Number> values) {
        for (Map.Entry<String, Number> entry : values.entrySet()) {
            NetworkTableEntry NTEntry = table.getEntry(entry.getKey());
            NTEntry.setDouble(entry.getValue().doubleValue());
        }
    }

    public static double getValue(String key) {
        return table.getEntry(key).getDouble(0.0);
    }

    public static DoubleSupplier getSupplier(String key) {
        return () -> getValue(key);
    }
}