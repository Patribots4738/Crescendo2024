package frc.robot.logging;

import monologue.Logged;
import monologue.Monologue;

/**
 * This class is used to configure the paths for the Monologue logging system.
 * 
 * Because we want to log all of the subsystems and managers, but to be logged in a custom path,
 * we need to configure the paths for each of the subsystems and managers.
 * 
 * We have to pass in the subsystems because Monologue is a singleton and we can't create a new instance of it.
 * The current instance of Monologue is run on RobotContainer, so we have to pass in the subsystems and managers
 * that are being initialized in RobotContainer.
 */
public class ConfigureMonologuePaths {
    int limelightLogged = 0;

    public ConfigureMonologuePaths(Logged[] classesToLog) {
        configureLoggingPaths(classesToLog);
    }

    private void logObj(Logged logged, String name) {
        String path = null;
        String path2 = null;

        switch (name) {
            case "CalibrationControl" -> {
                path = "Robot/Math/calibrationControl";
                path2 = "Robot/Managers/pieceControl";
            }

            case "HDCTuner" -> path = "Robot/Math/HDCTuner";
            case "PieceControl" -> path = "Robot/Math/PieceControl";

            case "Swerve" -> path = "Robot/Swerve";

            case "Intake" -> path = "Robot/Subsystems/intake";

            case "Climb" -> path = "Robot/Subsystems/climb";

            case "LimelightMapper" -> path = "Robot/Limelights/limelightMapper";
            case "Limelight" -> {
                limelightLogged++;
                if (limelightLogged == 1) {
                    path = "Robot/Limelights/limelight2";
                } else if (limelightLogged == 2) {
                    path = "Robot/Limelights/limelight3";
                }
            }

            case "ColorSensor" -> path = "Robot/ColorSensors/colorSensor";
            case "Shooter" -> path = "Robot/Subsystems/shooter";
            case "Elevator" -> path = "Robot/Subsystems/elevator";
            case "Pivot" -> path = "Robot/Subsystems/pivot";
            case "Ampper" -> path = "Robot/Subsystems/ampper";
            case "PathPlannerStorage" -> path = "Robot";
            default -> path = "SyncIssues";
        }
        if (path == "SyncIssues")
            System.out.println(
                "Failed to Log: "
                + name
                + " | If this is a new class, make sure to update the switch case in ConfigureMonologuePaths"
            );
        System.out.println("Logging of " + name + " to: '" + path + "' was successful ");
        Monologue.logObj(logged, path);

        if (path2 != null) {
            System.out.println("Logging of " + name + " to: '" + path2 + "' was successful ");
            Monologue.logObj(logged, path2);
        }
            
        System.out.println("\n");
    }

    private void configureLoggingPaths(Logged[] classesToLog) {
        for (Logged object : classesToLog) {
            if (object == null)
                continue;

            String name = object.getClass().getName();
            
            String[] splitName = name.split("\\.");

            if (splitName.length <= 1)
                continue;

            name = splitName[splitName.length - 1];
            System.out.println("Attempting to Log: " + name);

            logObj(object, name);
        }
    }

}
