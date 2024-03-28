package frc.robot.logging;

import frc.robot.Constants.CameraConstants;
import frc.robot.calc.PathPlannerStorage;
import frc.robot.calc.ShooterCalc;
import frc.robot.managers.CalibrationControl;
import frc.robot.managers.HDCTuner;
import frc.robot.managers.PieceControl;
import frc.robot.subsystems.Ampper;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import lib.limelightHelpers.LimelightMapping;
import monologue.Monologue;

public class ConfigureMonologuePaths {

    public ConfigureMonologuePaths(ShooterCalc shooterCalc,
            CalibrationControl calibrationControl,
            HDCTuner HDCTuner,
            PieceControl pieceControl,
            Swerve swerve,
            Intake intake,
            Climb climb,
            LimelightMapping limelightMapper,
            Limelight limelight2,
            Limelight limelight3,
            ColorSensor colorSensor,
            Shooter shooter,
            Elevator elevator,
            Pivot pivot,
            Ampper ampper,
            PathPlannerStorage pathPlannerStorage) {
        configureLoggingPaths(shooterCalc, calibrationControl, HDCTuner, pieceControl, swerve, intake, climb,
                limelightMapper, limelight2, limelight3, colorSensor, shooter, elevator, pivot, ampper,
                pathPlannerStorage);
    }

    private void configureLoggingPaths(ShooterCalc shooterCalc,
            CalibrationControl calibrationControl,
            HDCTuner HDCTuner,
            PieceControl pieceControl,
            Swerve swerve,
            Intake intake,
            Climb climb,
            LimelightMapping limelightMapper,
            Limelight limelight2,
            Limelight limelight3,
            ColorSensor colorSensor,
            Shooter shooter,
            Elevator elevator,
            Pivot pivot,
            Ampper ampper,
            PathPlannerStorage pathPlannerStorage) {
        Monologue.logObj(shooterCalc, "Robot/Math/shooterCalc");
        Monologue.logObj(calibrationControl, "Robot/Math/calibrationControl");
        Monologue.logObj(HDCTuner, "Robot/Math/HDCTuner");
        Monologue.logObj(pieceControl, "Robot/Math/PieceControl");

        Monologue.logObj(swerve, "Robot/Swerve");

        Monologue.logObj(intake, "Robot/Subsystems/intake");
        Monologue.logObj(climb, "Robot/Subsystems/climb");
        if (CameraConstants.FIELD_CALIBRATION_MODE) {
            Monologue.logObj(limelightMapper, "Robot/Limelights/limelightMapper");
        } else {
            Monologue.logObj(limelight2, "Robot/Limelights/limelight2");
            Monologue.logObj(limelight3, "Robot/Limelights/limelight3");
        }
        Monologue.logObj(colorSensor, "Robot/ColorSensors/colorSensor");
        Monologue.logObj(shooter, "Robot/Subsystems/shooter");
        Monologue.logObj(elevator, "Robot/Subsystems/elevator");
        Monologue.logObj(pivot, "Robot/Subsystems/pivot");
        Monologue.logObj(ampper, "Robot/Subsystems/ampper");
        Monologue.logObj(pieceControl, "Robot/Managers/pieceControl");

        Monologue.logObj(pathPlannerStorage, "Robot");
    }

}
