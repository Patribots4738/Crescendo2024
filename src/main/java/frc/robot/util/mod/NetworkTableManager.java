package frc.robot.util.mod;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import frc.robot.util.constants.Constants.AutoConstants;

public class NetworkTableManager {
    public class PatriNetworkTableLayout {
        public static final String ROBOT_TABLE = "Robot";

        public static final String XY_P_ENTRY_NAME = "Auto/Translation/P";
        public static final String XY_I_ENTRY_NAME = "Auto/Translation/I";
        public static final String XY_D_ENTRY_NAME = "Auto/Translation/D";
    
        public static final String ROTATION_P_ENTRY_NAME = "Auto/Rotation/P";
        public static final String ROTATION_I_ENTRY_NAME = "Auto/Rotation/I";
        public static final String ROTATION_D_ENTRY_NAME = "Auto/Rotation/D";

        public static final String MAX_ENTRY_NAME = "MAX";
        
        public static final Pair<String, String[]> ROBOT_MAP = new Pair<>(
            ROBOT_TABLE, 
            new String[] {
                XY_P_ENTRY_NAME,
                XY_I_ENTRY_NAME,
                XY_D_ENTRY_NAME,
                ROTATION_P_ENTRY_NAME,
                ROTATION_I_ENTRY_NAME,
                ROTATION_D_ENTRY_NAME
            }
        );
    }

    public NetworkTableManager() {
        HashMap<String, Double> map = new HashMap<>(){{
            put(PatriNetworkTableLayout.XY_D_ENTRY_NAME, -1.0);
            put(PatriNetworkTableLayout.XY_I_ENTRY_NAME, -1.0);
            put(PatriNetworkTableLayout.XY_D_ENTRY_NAME, -1.0);
            put(PatriNetworkTableLayout.ROTATION_P_ENTRY_NAME, -1.0);
            put(PatriNetworkTableLayout.ROTATION_I_ENTRY_NAME, AutoConstants.ROTATION_CORRECTION_I);
            put(PatriNetworkTableLayout.ROTATION_D_ENTRY_NAME, -1.0);
        }};
        setPatriNetworkTableLayout(map);
    }

    public <T> HashMap<String, T> getPatriNetworkTableLayout() {
        HashMap<String, T> map = new HashMap<>();
        for (String entryName : PatriNetworkTableLayout.ROBOT_MAP.getSecond()) {
            NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable(
                PatriNetworkTableLayout.ROBOT_MAP.getFirst())
                .getEntry(entryName);

            map.put(entryName, getEntryByType(entry));
        }
        return map;
    }

    @SuppressWarnings("unchecked")
    public <T> T getEntryByType(NetworkTableEntry entry) {
        NetworkTableType type = entry.getType();
        switch (type) {
            case kBoolean:
                return (T) (Boolean) entry.getBoolean(false);
            case kDouble:
                return (T) (Double) entry.getDouble(-1.0);
            case kString:
                return (T) (String) entry.getString("");
            case kInteger:
                return (T) (Long) entry.getInteger(-1);
            case kBooleanArray:
                return (T) (boolean[]) entry.getBooleanArray(new boolean[0]);
            case kDoubleArray:
                return (T) (double[]) entry.getDoubleArray(new double[0]);
            case kStringArray:
                return (T) (String[]) entry.getStringArray(new String[0]);
            case kIntegerArray:
                return (T) (long[]) entry.getIntegerArray(new long[0]);
            default:
                return null;
        }
    }

    public <T> void setEntryByType(NetworkTableEntry entry, T value) {
        NetworkTableType type = entry.getType();
        switch (type) {
            case kBoolean:
                entry.setBoolean((Boolean) value);
                break;
            case kDouble:
                if ((Double) value == -1.0) return;
                entry.setDouble((Double) value);
                break;
            case kString:
                entry.setString((String) value);
                break;
            case kInteger:
                entry.setInteger((Long) value);
                break;
            case kBooleanArray:
                entry.setBooleanArray((boolean[]) value);
                break;
            case kDoubleArray:
                entry.setDoubleArray((double[]) value);
                break;
            case kStringArray:
                entry.setStringArray((String[]) value);
                break;
            case kIntegerArray:
                entry.setIntegerArray((long[]) value);
                break;
            default:
                break;
        }
    }

    public <T> void setPatriNetworkTableLayout(HashMap<String, T> map) {
        for (String entryName : PatriNetworkTableLayout.ROBOT_MAP.getSecond()) {
            NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable(
                PatriNetworkTableLayout.ROBOT_MAP.getFirst())
                .getEntry(entryName);
            setEntryByType(entry, map.get(entryName));
        }
    }

    public <T> HashMap<String, T> updatePatriNetworkTableLayout() {
        setPatriNetworkTableLayout(getPatriNetworkTableLayout());
        return getPatriNetworkTableLayout();
    }
}
