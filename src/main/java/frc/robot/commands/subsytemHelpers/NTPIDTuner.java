package frc.robot.commands.subsytemHelpers;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Map.Entry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants.NeoMotorConstants;
import frc.robot.util.rev.Neo;

public class NTPIDTuner extends Command {
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Calibration");
    Map<List<Neo>, Map<NetworkTableEntry, Double>> entries = new HashMap<>();

    public NTPIDTuner() {
        // Initialize the motor group map
        Map<String, List<Neo>> motorGroups = NeoMotorConstants.initializeMotorGroupMap();

        // Initialize the NetworkTableEntries for the PID values of each motor group
        initializeMotorEntries(motorGroups);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private void initializeMotorEntries(Map<String, List<Neo>> motorGroups) {
        // Loop through each motor group
        for (int i = 0; i < motorGroups.size(); i++) {

            // Get the name of the current motor group
            String groupName = motorGroups.keySet().toArray()[i].toString();

            // Get the NetworkTableEntries for the PID values of the current motor group
            NetworkTableEntry pEntry = table.getEntry(groupName + "/0-P");
            NetworkTableEntry iEntry = table.getEntry(groupName + "/1-I");
            NetworkTableEntry dEntry = table.getEntry(groupName + "/2-D");
            NetworkTableEntry ffEntry = table.getEntry(groupName + "/3-FF");
            NetworkTableEntry iZoneEntry = table.getEntry(groupName + "/4-IZone");

            // Get the first motor of the current motor group
            List<Neo> motorGroup = motorGroups.get(groupName);
            Neo motor = motorGroup.get(0);

            // Get the PID values of the first motor
            double pValue = motor.getP();
            double iValue = motor.getI();
            double dValue = motor.getD();
            double ffValue = motor.getFF();
            double iZoneValue = motor.getIZone();

            // Set the PID values in the NetworkTableEntries
            pEntry.setDouble(pValue);
            iEntry.setDouble(iValue);
            dEntry.setDouble(dValue);
            ffEntry.setDouble(ffValue);
            iZoneEntry.setDouble(iZoneValue);

            // Create a map of NetworkTableEntries and their corresponding PID values
            Map<NetworkTableEntry, Double> motorEntries = new HashMap<>();
            motorEntries.put(pEntry, pValue);
            motorEntries.put(iEntry, iValue);
            motorEntries.put(dEntry, dValue);
            motorEntries.put(ffEntry, ffValue);
            motorEntries.put(iZoneEntry, iZoneValue);

            // Add the motor group and its corresponding NetworkTableEntries to the entries map
            entries.put(motorGroup, motorEntries);
        }
    }
    
    @Override
    public void execute() {
        // Loop through all of the entries and update the values
        for (Map.Entry<List<Neo>, Map<NetworkTableEntry, Double>> entry : entries.entrySet()) {
            updateValues(entry);
        }
    }

    private void updateValues(Map.Entry<List<Neo>, Map<NetworkTableEntry, Double>> entry) {
        for (Map.Entry<NetworkTableEntry, Double> entry2 : entry.getValue().entrySet()) {
            double NTValue = entry2.getKey().getDouble(entry2.getValue());
            if (NTValue != entry2.getValue()) {
                String entryPath = entry2.getKey().getName();
                // We are given a full path like "/Calibration//FrontLeftTurn/3-FF"
                // We just want that "FF" part
                String lastPart = entryPath.substring(entryPath.lastIndexOf("-") + 1);
                
                if (lastPart.equals("IZone")) {
                    if (NTValue > 99) {
                        NTValue = Double.POSITIVE_INFINITY;
                    } else if (NTValue < 0) {
                        NTValue = 0;
                    }
                    if (NTValue == entry2.getValue()) {
                        return;
                    }
                    entry2.setValue(NTValue);
                }

                // Update the last known value for future comparisons
                entry2.setValue(NTValue);
                updateController(entry.getKey(), lastPart, NTValue);
            }
        }
    }

    private void updateController(List<Neo> motorList, String name, double NTValue) {
        // Find out if it was P, I, D, FF, or IZone and update the value
        for (Neo motor : motorList) {
            switch (name) {
                case "P":
                    motor.setP(NTValue);
                    break;
                case "I":
                    motor.setI(NTValue);
                    break;
                case "D":
                    motor.setD(NTValue);
                    break;
                case "FF":
                    motor.setFF(NTValue);
                    break;
                case "IZone":
                    motor.setIZone(NTValue);
                    break;
            }
        }
        System.err.println("Updated " + getKeyByValue(NeoMotorConstants.MOTOR_GROUPS, motorList) + "'s " + name + " to " + NTValue);
    }

    /**
     * Retrieves the key associated with a given value in a map.
     *
     * @param map   the map to search in
     * @param value the value to search for
     * @return the key associated with the given value, or null if not found
     */
    private <T, E> T getKeyByValue(Map<T, E> map, E value) {
        for (Entry<T, E> entry : map.entrySet()) {
            if (Objects.equals(value, entry.getValue())) {
                return entry.getKey();
            }
        }
        return null;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
