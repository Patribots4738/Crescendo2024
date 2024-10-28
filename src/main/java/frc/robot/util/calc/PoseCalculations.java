package frc.robot.util.calc;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.ClimbConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.ShooterConstants;

public class PoseCalculations {

    /**
     * Calculates the intercepts of the chain based on the given robot position.
     * 
     * @param position the robot's position
     * @return a Pair containing the left and right intercepts of the chain
     */
    public static Pair<Double, Double> getChainIntercepts(Pose2d position) {
        Pose2d closestChainPose = getClosestChain(position);

        Pose2d relativePosition = position.relativeTo(closestChainPose);

        if (Math.abs(relativePosition.getTranslation().getX()) > 1) {
            return Pair.of(0d, 0d);
        }

        double leftIntercept = getChainIntercept(relativePosition.getY() + ClimbConstants.DISTANCE_FROM_ORIGIN_METERS);
        double rightIntercept = getChainIntercept(relativePosition.getY() - ClimbConstants.DISTANCE_FROM_ORIGIN_METERS);

        return Pair.of(leftIntercept - 0.45, rightIntercept - 0.45);
    }

    /**
     * Calculates the closest chain position to a given pose.
     *
     * @param position The pose for which to find the closest chain position.
     * @return The closest chain position to the given pose.
     */
    public static Pose2d getClosestChain(Pose2d position) {
        return position.nearest(FieldConstants.GET_CHAIN_POSITIONS());
    }

    @AutoLogOutput (key = "Calc/PoseCalc/ClosestShootingPose")
    private static Pose2d closestShootingPose;

    @AutoLogOutput (key = "Calc/PoseCalc/BestShootingPose")
    public static Pose2d getBestShootingPose(Pose2d position) {
        double yValue = position.getY();
        if (yValue > 5)
            // L
            return FieldConstants.GET_SHOOTING_POSITIONS().get(0);
        if (yValue > 1.8) 
            // M
            return FieldConstants.GET_SHOOTING_POSITIONS().get(2);
        // R
        return FieldConstants.GET_SHOOTING_POSITIONS().get(1);
    }

    public static Pose2d getClosestShootingPose(Pose2d pose) {
        return pose.nearest(FieldConstants.PRESET_SHOT_POSITIONS);                                                                                                                                                                                                                                                                                                                                                                           
    }

    public static String getBestShootingPoseString(Pose2d position) {
        closestShootingPose = getBestShootingPose(position);
        if (closestShootingPose.equals(FieldConstants.GET_SHOOTING_POSITIONS().get(0)))
            return "L";

        if (closestShootingPose.equals(FieldConstants.GET_SHOOTING_POSITIONS().get(1)))
            return "R";

        if (closestShootingPose.equals(FieldConstants.GET_SHOOTING_POSITIONS().get(2)))
            return "M";
        
        return "if you managed to get this to run you deserve a cookie... wait.. no. i want that cookie.";
    }

    /**
     * Given an intercept of the chain, return the height of the chain at that
     * location.
     * 
     * @param x The position along the X axis of the chain
     *          as seen in https://www.desmos.com/calculator/84ioficbl2
     *          For some reason, that desmos breaks on chrome on my home computer
     *          please send help... i used edge to make it :(
     * 
     * @return The height of the chain at that location
     */
    private static double getChainIntercept(double x) {
        // The ds here turn the integers into doubles
        // so that integer division does not occur.
        double calculation = 3d / 10d * Math.pow(x, 2) + 0.725;
        // Clamp the output to be no lower than the lowest point of the chain,
        // and no higher than the extension limit of our elevator
        return MathUtil.clamp(calculation, 0.725, ClimbConstants.EXTENSION_LIMIT_METERS);
    }

    /**
     * Detects if the robot is in the opposite wing
     * Uses the robot's x position to determine if it has crossed the opp wing line.
     * 
     * @return true if the robot is in the opposing wing
     */
    public static boolean onOppositeWing(double xPosition) {
        return Robot.isRedAlliance() 
            ? xPosition < FieldConstants.BLUE_WING_X 
            : xPosition > FieldConstants.RED_WING_X;
    }

    /**
     * Detects if the robot is on the opposite side of the field.
     * Uses the robot's x position to determine if it has crossed the centerline.
     * 
     * @return true if the robot is on the opposite side of the field
     */
    public static boolean onOppositeSide(double xPosition) {
        return Robot.isRedAlliance() 
            ? xPosition < FieldConstants.CENTERLINE_X 
            : xPosition > FieldConstants.CENTERLINE_X;
    }

    // Note: this method uses a static variable rather than a parameter
    public static boolean closeToSpeaker() {
        return RobotContainer.distanceToSpeakerFeet <= ShooterConstants.TUNED_SHOOTER_MAX_DISTANCE;
    }

    public static boolean inSpeakerShotZone(Translation2d translation) {
        Translation2d stealBoxCorner = FieldConstants.GET_STEAL_BOX_CORNER();
        return 
            closeToSpeaker() 
            && ((Robot.isRedAlliance() 
                    ? translation.getX() < stealBoxCorner.getX() 
                    : translation.getX() > stealBoxCorner.getX()) 
                || translation.getY() > stealBoxCorner.getY());
    }

    /**
     * Detects if the robot is inside of the area of the stage
     * 
     * @return true if the robot is inside of the area of the stage
     */
    public static boolean inStageTriangle(Pose2d position) {
        
        List<Translation2d> points = FieldConstants.GET_STAGE_POINTS(); 

        double a1 = points.get(0).minus(position.getTranslation()).getNorm();
        double b1 = points.get(2).minus(position.getTranslation()).getNorm();
        double c1 = points.get(2).minus(points.get(0)).getNorm();
        double s1 = 0.5 * (a1 + b1 + c1);

        double a2 = a1;
        double b2 = points.get(1).minus(position.getTranslation()).getNorm();
        double c2 = points.get(1).minus(points.get(0)).getNorm();
        double s2 = 0.5 * (a2 + b2 + c2);

        double a3 = b2;
        double b3 = b1;
        double c3 = points.get(2).minus(points.get(1)).getNorm();
        double s3 = 0.5 * (a3 + b3 + c3);

        double mainArea = 0.5 * Math.abs(points.get(2).getY() - points.get(1).getY()) * Math.abs(points.get(1).getX() - points.get(0).getX());
        double area1 = Math.sqrt(s1 * (s1 - a1) * (s1 - b1) * (s1 - c1));
        double area2 = Math.sqrt(s2 * (s2 - a2) * (s2 - b2) * (s2 - c2));
        double area3 = Math.sqrt(s3 * (s3 - a3) * (s3 - b3) * (s3 - c3));

        return MathUtil.applyDeadband(mainArea - (area1 + area2 + area3), AutoConstants.UNDER_STAGE_DEADBAND) == 0;
    }

    public static boolean isAlignedToAmp(Pose2d position) {
        Translation2d touchingAmpPose = new Translation2d(
            FieldConstants.GET_AMP_POSITION().getX(),
            FieldConstants.GET_AMP_POSITION().getY() 
                - DriveConstants.ROBOT_LENGTH_METERS / 2.0
                - DriveConstants.BUMPER_LENGTH_METERS
        );
        double robotX = position.getTranslation().getDistance(touchingAmpPose);
        return MathUtil.isNear(0, robotX, AutoConstants.AUTO_ALIGNMENT_DEADBAND);
    }

    public static boolean inSourcePassZone(Pose2d position) {
        return 
            Robot.isRedAlliance()
                ? position.getX() < FieldConstants.BLUE_WING_X
                : position.getX() > FieldConstants.RED_WING_X;
    }

    public static boolean inLowPassZone(Pose2d position) {
        return 
            position.getY() > FieldConstants.GET_STAGE_POINTS().get(2).getY()
            || (Robot.isRedAlliance() 
                ? position.getX() > FieldConstants.NOTE_TRANSLATIONS[3].getX() 
                : position.getX() < FieldConstants.NOTE_TRANSLATIONS[0].getX());
    }

}