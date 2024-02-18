//LimelightHelpers v1.2.1 (March 1, 2023)

package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

public class LimelightHelpers {

    /**
     * Represents a retroreflective target detected by the Limelight camera.
     */
    public static class LimelightTarget_Retro {

        /**
         * The camera pose in target space.
         */
        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        /**
         * The robot pose in field space.
         */
        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        /**
         * The robot pose in target space.
         */
        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        /**
         * The target pose in camera space.
         */
        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        /**
         * The target pose in robot space.
         */
        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        /**
         * Gets the camera pose in target space as a Pose3d object.
         *
         * @return The camera pose in target space.
         */
        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        /**
         * Gets the robot pose in field space as a Pose3d object.
         *
         * @return The robot pose in field space.
         */
        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        /**
         * Gets the robot pose in target space as a Pose3d object.
         *
         * @return The robot pose in target space.
         */
        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        /**
         * Gets the target pose in camera space as a Pose3d object.
         *
         * @return The target pose in camera space.
         */
        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        /**
         * Gets the target pose in robot space as a Pose3d object.
         *
         * @return The target pose in robot space.
         */
        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        /**
         * Gets the camera pose in target space as a Pose2d object.
         *
         * @return The camera pose in target space.
         */
        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        /**
         * Gets the robot pose in field space as a Pose2d object.
         *
         * @return The robot pose in field space.
         */
        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        /**
         * Gets the robot pose in target space as a Pose2d object.
         *
         * @return The robot pose in target space.
         */
        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        /**
         * Gets the target pose in camera space as a Pose2d object.
         *
         * @return The target pose in camera space.
         */
        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        /**
         * Gets the target pose in robot space as a Pose2d object.
         *
         * @return The target pose in robot space.
         */
        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        /**
         * The area of the target.
         */
        @JsonProperty("ta")
        public double ta;

        /**
         * The horizontal offset from the crosshair to the target.
         */
        @JsonProperty("tx")
        public double tx;

        /**
         * The horizontal offset from the crosshair to the target in pixels.
         */
        @JsonProperty("txp")
        public double tx_pixels;

        /**
         * The vertical offset from the crosshair to the target.
         */
        @JsonProperty("ty")
        public double ty;

        /**
         * The vertical offset from the crosshair to the target in pixels.
         */
        @JsonProperty("typ")
        public double ty_pixels;

        /**
         * The skew or rotation of the target.
         */
        @JsonProperty("ts")
        public double ts;

        /**
         * Creates a new instance of LimelightTarget_Retro.
         */
        public LimelightTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }

    }

    /**
     * Represents a target detected by the Limelight camera with fiducial
     * information.
     */
    public static class LimelightTarget_Fiducial {

        /**
         * The ID of the fiducial.
         */
        @JsonProperty("fID")
        public double fiducialID;

        /**
         * The family of the fiducial.
         */
        @JsonProperty("fam")
        public String fiducialFamily;

        /**
         * The camera pose in target space.
         */
        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        /**
         * The robot pose in field space.
         */
        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        /**
         * The robot pose in target space.
         */
        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        /**
         * The target pose in camera space.
         */
        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        /**
         * The target pose in robot space.
         */
        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        /**
         * Get the camera pose in target space as a Pose3d object.
         *
         * @return The camera pose in target space.
         */
        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        /**
         * Get the robot pose in field space as a Pose3d object.
         *
         * @return The robot pose in field space.
         */
        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        /**
         * Get the robot pose in target space as a Pose3d object.
         *
         * @return The robot pose in target space.
         */
        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        /**
         * Get the target pose in camera space as a Pose3d object.
         *
         * @return The target pose in camera space.
         */
        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        /**
         * Get the target pose in robot space as a Pose3d object.
         *
         * @return The target pose in robot space.
         */
        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        /**
         * Get the camera pose in target space as a Pose2d object.
         *
         * @return The camera pose in target space.
         */
        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        /**
         * Get the robot pose in field space as a Pose2d object.
         *
         * @return The robot pose in field space.
         */
        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        /**
         * Get the robot pose in target space as a Pose2d object.
         *
         * @return The robot pose in target space.
         */
        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        /**
         * Get the target pose in camera space as a Pose2d object.
         *
         * @return The target pose in camera space.
         */
        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        /**
         * Get the target pose in robot space as a Pose2d object.
         *
         * @return The target pose in robot space.
         */
        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        /**
         * The area of the target.
         */
        @JsonProperty("ta")
        public double ta;

        /**
         * The x-axis offset of the target.
         */
        @JsonProperty("tx")
        public double tx;

        /**
         * The x-axis offset of the target in pixels.
         */
        @JsonProperty("txp")
        public double tx_pixels;

        /**
         * The y-axis offset of the target.
         */
        @JsonProperty("ty")
        public double ty;

        /**
         * The y-axis offset of the target in pixels.
         */
        @JsonProperty("typ")
        public double ty_pixels;

        /**
         * The skew or rotation of the target.
         */
        @JsonProperty("ts")
        public double ts;

        /**
         * Create a new instance of LimelightTarget_Fiducial.
         */
        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    // N/A
    public static class LimelightTarget_Barcode {

    }

    /**
     * Represents a target detected by the Limelight classifier.
     */
    public static class LimelightTarget_Classifier {

        /**
         * The class name of the target.
         */
        @JsonProperty("class")
        public String className;

        /**
         * The ID of the target class.
         */
        @JsonProperty("classID")
        public double classID;

        /**
         * The confidence level of the target classification.
         */
        @JsonProperty("conf")
        public double confidence;

        /**
         * The zone of the target.
         */
        @JsonProperty("zone")
        public double zone;

        /**
         * The horizontal offset from the crosshair to the target.
         */
        @JsonProperty("tx")
        public double tx;

        /**
         * The horizontal offset from the crosshair to the target in pixels.
         */
        @JsonProperty("txp")
        public double tx_pixels;

        /**
         * The vertical offset from the crosshair to the target.
         */
        @JsonProperty("ty")
        public double ty;

        /**
         * The vertical offset from the crosshair to the target in pixels.
         */
        @JsonProperty("typ")
        public double ty_pixels;

        /**
         * Constructs a new instance of the LimelightTarget_Classifier class.
         */
        public LimelightTarget_Classifier() {
        }
    }

    /**
     * Represents a detected target by the Limelight camera.
     */
    /**
     * Represents a detected target from the Limelight camera.
     */
    public static class LimelightTarget_Detector {

        /**
         * The class name of the detected target.
         */
        @JsonProperty("class")
        public String className;

        /**
         * The class ID of the detected target.
         */
        @JsonProperty("classID")
        public double classID;

        /**
         * The confidence level of the detected target.
         */
        @JsonProperty("conf")
        public double confidence;

        /**
         * The area of the detected target.
         */
        @JsonProperty("ta")
        public double ta;

        /**
         * The horizontal angle to the detected target.
         */
        @JsonProperty("tx")
        public double tx;

        /**
         * The horizontal angle to the detected target in pixels.
         */
        @JsonProperty("txp")
        public double tx_pixels;

        /**
         * The vertical angle to the detected target.
         */
        @JsonProperty("ty")
        public double ty;

        /**
         * The vertical angle to the detected target in pixels.
         */
        @JsonProperty("typ")
        public double ty_pixels;

        /**
         * Default constructor for the LimelightTarget_Detector class.
         */
        public LimelightTarget_Detector() {
        }
    }

    /**
     * Represents the results obtained from the Limelight camera.
     */
    public static class Results {

        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("tl")
        public double latency_pipeline;

        @JsonProperty("cl")
        public double latency_capture;

        public double latency_jsonParse;

        @JsonProperty("ts")
        public double timestamp_LIMELIGHT_publish;

        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        @JsonProperty("v")
        @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        @JsonProperty("botpose")
        public double[] botpose;

        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        @JsonProperty("t6c_rs")
        public double[] camerapose_robotspace;

        public Pose3d getBotPose3d() {
            return toPose3D(botpose);
        }

        public Pose3d getBotPose3d_wpiRed() {
            return toPose3D(botpose_wpired);
        }

        public Pose3d getBotPose3d_wpiBlue() {
            return toPose3D(botpose_wpiblue);
        }

        public Pose2d getBotPose2d() {
            return toPose2D(botpose);
        }

        public Pose2d getBotPose2d_wpiRed() {
            return toPose2D(botpose_wpired);
        }

        public Pose2d getBotPose2d_wpiBlue() {
            return toPose2D(botpose_wpiblue);
        }

        @JsonProperty("Retro")
        public LimelightTarget_Retro[] targets_Retro;

        @JsonProperty("Fiducial")
        public LimelightTarget_Fiducial[] targets_Fiducials;

        @JsonProperty("Classifier")
        public LimelightTarget_Classifier[] targets_Classifier;

        @JsonProperty("Detector")
        public LimelightTarget_Detector[] targets_Detector;

        @JsonProperty("Barcode")
        public LimelightTarget_Barcode[] targets_Barcode;

        public Results() {
            botpose = new double[6];
            botpose_wpired = new double[6];
            botpose_wpiblue = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro = new LimelightTarget_Retro[0];
            targets_Fiducials = new LimelightTarget_Fiducial[0];
            targets_Classifier = new LimelightTarget_Classifier[0];
            targets_Detector = new LimelightTarget_Detector[0];
            targets_Barcode = new LimelightTarget_Barcode[0];

        }
    }

    // The storage of the Results object
    public static class LimelightResults {
        /**
         * Represents the results of targeting using the Limelight.
         */
        @JsonProperty("Results")
        public Results targetingResults;

        /**
         * Constructs a new instance of the LimelightResults class.
         * Initializes the targetingResults field with a new instance of the Results
         * class.
         */
        public LimelightResults() {
            targetingResults = new Results();
        }
    }

    private static ObjectMapper mapper;

    /**
     * Print JSON Parse time to the console in milliseconds
     */
    static boolean profileJSON = false;

    /**
     * Sanitizes the given name by checking if it is empty or null.
     * If the name is empty or null, it returns "limelight".
     * Otherwise, it returns the original name.
     *
     * @param name the name to be sanitized
     * @return the sanitized name
     */
    static final String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "limelight";
        }
        return name;
    }

    /**
     * Converts an array of double values to a Pose3d object.
     * If the input data is invalid, a default Pose3d object is returned.
     * 
     * @param inData the input array of double values representing the pose data
     * @return a Pose2d object representing the converted pose data
     */
    private static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
                new Translation3d(inData[0], inData[1], inData[2]),
                new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                        Units.degreesToRadians(inData[5])));
    }

    /**
     * Converts an array of double values to a Pose2d object.
     * If the input data is invalid, a default Pose2d object is returned.
     * 
     * @param inData the input array of double values representing the pose data
     * @return a Pose2d object representing the converted pose data
     */
    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    /**
     * Retrieves the NetworkTable associated with the specified table name.
     *
     * @param tableName the name of the NetworkTable to retrieve
     * @return the NetworkTable instance
     */
    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    /**
     * Retrieves the NetworkTableEntry object for the specified table and entry
     * names.
     *
     * @param tableName The name of the NetworkTable.
     * @param entryName The name of the NetworkTableEntry.
     * @return The NetworkTableEntry object.
     */
    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    /**
     * Retrieves the value of a double entry from the specified NetworkTable in the
     * Limelight.
     * 
     * @param tableName the name of the NetworkTable
     * @param entryName the name of the entry
     * @return the value of the double entry
     */
    public static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    /**
     * Sets the value of a double entry in the NetworkTables for the specified table
     * and entry names.
     *
     * @param tableName the name of the NetworkTables table
     * @param entryName the name of the NetworkTables entry
     * @param val       the value to set for the double entry
     */
    public static void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    /**
     * Sets the value of a NetworkTable entry as a double array in the specified
     * table.
     *
     * @param tableName the name of the NetworkTable
     * @param entryName the name of the NetworkTable entry
     * @param val       the double array value to be set
     */
    public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    /**
     * Retrieves a double array value from the NetworkTables using the specified
     * table name and entry name.
     * If the value is not found, it returns an empty double array.
     *
     * @param tableName the name of the NetworkTables table
     * @param entryName the name of the NetworkTables entry
     * @return the double array value from the NetworkTables, or an empty double
     *         array if not found
     */
    public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }

    /**
     * Retrieves a string value from the NetworkTables using the specified table
     * name and entry name.
     *
     * @param tableName the name of the NetworkTables table
     * @param entryName the name of the NetworkTables entry
     * @return the string value of the NetworkTables entry
     */
    public static String getLimelightNTString(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    /**
     * Returns a URL object representing the Limelight URL string for the local ip.
     *
     * @param tableName the name of the Limelight table
     * @param request   the request to be appended to the URL
     * @return a URL object representing the Limelight URL string
     */
    public static URL getLimelightURLString(String tableName, String request) {
        String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
        URL url;
        try {
            url = new URL(urlString);
            return url;
        } catch (MalformedURLException e) {
            System.err.println("bad LL URL");
        }
        return null;
    }
    /////
    /////

    /**
     * Returns the horizontal angle offset (TX) from the Limelight camera with the
     * specified name.
     *
     * @param limelightName the name of the Limelight camera
     * @return the horizontal angle offset (TX) in degrees
     */
    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    /**
     * Returns the vertical angle of the target in degrees.
     * 
     * @param limelightName the name of the Limelight camera
     * @return the vertical angle of the target in degrees
     */
    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    /**
     * Retrieves the target area (TA) value from the specified Limelight.
     * 
     * @param limelightName the name of the Limelight
     * @return the target area value
     */
    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    /**
     * Returns the latency of the specified Limelight camera in milliseconds.
     * 
     * @param limelightName the name of the Limelight camera
     * @return the latency of the Limelight camera in milliseconds
     */
    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    /**
     * Returns the latency of the Limelight camera capture.
     * 
     * @param limelightName the name of the Limelight camera
     * @return the latency of the camera capture in seconds
     */
    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }

    /**
     * Retrieves the current pipeline index of the specified Limelight.
     * 
     * @param limelightName the name of the Limelight
     * @return the current pipeline index
     */
    public static double getCurrentPipelineIndex(String limelightName) {
        return getLimelightNTDouble(limelightName, "getpipe");
    }

    /**
     * Retrieves the JSON dump of the specified Limelight.
     *
     * @param limelightName the name of the Limelight
     * @return the JSON dump as a string
     */
    public static String getJSONDump(String limelightName) {
        return getLimelightNTString(limelightName, "json");
    }

    /**
     * Switch to getBotPose
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    /**
     * Switch to getBotPose_wpiRed
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    /**
     * Switch to getBotPose_wpiBlue
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    /**
     * Retrieves the pose of the robot from the specified Limelight camera.
     * 
     * @param limelightName the name of the Limelight camera
     * @return an array containing the X, Y, and Z coordinates of the robot's pose
     */
    public static double[] getBotPose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    /**
     * Retrieves the bot pose in the world coordinate system using the WPI Red
     * algorithm from the specified Limelight.
     *
     * @param limelightName the name of the Limelight
     * @return an array containing the bot pose in the world coordinate system [x,
     *         y, rotation]
     */
    public static double[] getBotPose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    /**
     * Retrieves the bot pose in the WPI Blue format from the specified Limelight.
     * 
     * @param limelightName the name of the Limelight
     * @return an array containing the bot pose in the WPI Blue format
     */
    public static double[] getBotPose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    /**
     * Retrieves the bot pose in target space from the specified Limelight.
     *
     * @param limelightName the name of the Limelight
     * @return an array containing the bot pose in target space
     */
    public static double[] getBotPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    }

    /**
     * Retrieves the camera pose in target space from the specified Limelight.
     *
     * @param limelightName the name of the Limelight
     * @return an array containing the camera pose in target space
     */
    public static double[] getCameraPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    /**
     * Retrieves the target pose in camera space from the specified Limelight.
     * 
     * @param limelightName the name of the Limelight
     * @return an array containing the target pose in camera space
     */
    public static double[] getTargetPose_CameraSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    /**
     * Retrieves the target pose in robot space from the specified Limelight.
     *
     * @param limelightName the name of the Limelight
     * @return an array containing the target pose in robot space
     */
    public static double[] getTargetPose_RobotSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    /**
     * Retrieves the target color from the specified Limelight.
     *
     * @param limelightName the name of the Limelight
     * @return an array of doubles representing the target color
     */
    public static double[] getTargetColor(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    /**
     * Retrieves the fiducial ID from the specified Limelight.
     *
     * @param limelightName the name of the Limelight
     * @return the fiducial ID
     */
    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    /**
     * Retrieves the neural class ID from the specified Limelight.
     *
     * @param limelightName the name of the Limelight
     * @return the neural class ID
     */
    public static double getNeuralClassID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tclass");
    }

    /////
    /////

    /**
     * Retrieves the pose of the robot in 3D space.
     * 
     * @param limelightName the name of the Limelight camera
     * @return the pose of the robot as a Pose3d object
     */
    public static Pose3d getBotPose3d(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
        return toPose3D(poseArray);
    }

    /**
     * Retrieves the pose of the robot in a 3D coordinate system with respect to the
     * WPI Red field using the specified Limelight.
     * 
     * @param limelightName the name of the Limelight camera
     * @return the pose of the robot in a 3D coordinate system
     */
    public static Pose3d getBotPose3d_wpiRed(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
        return toPose3D(poseArray);
    }

    /**
     * Retrieves the pose of the robot in a 3D coordinate system with respect to the
     * WPI Blue Alliance field.
     * 
     * @param limelightName the name of the Limelight camera
     * @return the pose of the robot in a Pose3d object
     */
    public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        return toPose3D(poseArray);
    }

    /**
     * Retrieves the pose of the robot in the target space from the specified
     * Limelight.
     *
     * @param limelightName the name of the Limelight camera
     * @return the pose of the robot in the target space as a Pose3d object
     */
    public static Pose3d getBotPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
        return toPose3D(poseArray);
    }

    /**
     * Retrieves the camera pose in target space for the specified Limelight.
     * 
     * @param limelightName the name of the Limelight
     * @return the camera pose in target space as a Pose3d object
     */
    public static Pose3d getCameraPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
        return toPose3D(poseArray);
    }

    /**
     * Retrieves the pose of the target in camera space.
     * 
     * @param limelightName the name of the Limelight camera
     * @return the pose of the target in camera space as a Pose3d object
     */
    public static Pose3d getTargetPose3d_CameraSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
        return toPose3D(poseArray);
    }

    /**
     * Retrieves the pose of the target in robot space from the specified Limelight.
     * 
     * @param limelightName the name of the Limelight
     * @return the pose of the target in robot space as a Pose3d object
     */
    public static Pose3d getTargetPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Returns the camera pose in robot space as a Pose3d object.
     * 
     * @param limelightName the name of the Limelight camera
     * @return the camera pose in robot space as a Pose3d object
     */
    public static Pose3d getCameraPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {

        double[] result = getBotPose_wpiBlue(limelightName);
        return toPose2D(result);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public static Pose2d getBotPose2d_wpiRed(String limelightName) {

        double[] result = getBotPose_wpiRed(limelightName);
        return toPose2D(result);

    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public static Pose2d getBotPose2d(String limelightName) {

        double[] result = getBotPose(limelightName);
        return toPose2D(result);

    }

    /**
     * Returns whether the Limelight's vision target is valid.
     * 
     * @param limelightName the name of the Limelight
     * @return true if the vision target is valid, false otherwise
     */
    public static boolean getTV(String limelightName) {
        return 1.0 == getLimelightNTDouble(limelightName, "tv");
    }

    /////
    /////

    /**
     * Sets the pipeline index for the specified Limelight.
     * 
     * @param limelightName the name of the Limelight
     * @param pipelineIndex the index of the pipeline to set
     */
    public static void setPipelineIndex(String limelightName, int pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    /**
     * The LEDs will be controlled by Limelight pipeline settings, and not by robot
     * code.
     */
    public static void setLEDMode_PipelineControl(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    /**
     * Sets the LED mode of the specified Limelight to force off.
     * 
     * @param limelightName the name of the Limelight
     */
    public static void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    /**
     * Sets the LED mode of the Limelight to force blink.
     * 
     * @param limelightName the name of the Limelight
     */
    public static void setLEDMode_ForceBlink(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    /**
     * Sets the LED mode of the specified Limelight to force on.
     * 
     * @param limelightName the name of the Limelight
     */
    public static void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    /**
     * Sets the stream mode of the specified Limelight to standard mode.
     * 
     * @param limelightName the name of the Limelight
     */
    public static void setStreamMode_Standard(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    /**
     * Sets the stream mode of the specified Limelight to PiP Main.
     * 
     * @param limelightName the name of the Limelight
     */
    public static void setStreamMode_PiPMain(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    /**
     * Sets the stream mode of the specified Limelight to PiP Secondary.
     * 
     * @param limelightName the name of the Limelight
     */
    public static void setStreamMode_PiPSecondary(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 2);
    }

    /**
     * Sets the camera mode of the specified Limelight to Processor mode.
     * 
     * @param limelightName the name of the Limelight
     */
    public static void setCameraMode_Processor(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 0);
    }

    /**
     * Sets the camera mode of the specified Limelight to driver mode.
     * 
     * @param limelightName the name of the Limelight
     */
    public static void setCameraMode_Driver(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 1);
    }

    /**
     * Sets the crop window. The crop window in the UI must be completely open for
     * dynamic cropping to work.
     */
    public static void setCropWindow(String limelightName, double cropXMin, double cropXMax, double cropYMin,
            double cropYMax) {
        double[] entries = new double[4];
        entries[0] = cropXMin;
        entries[1] = cropXMax;
        entries[2] = cropYMin;
        entries[3] = cropYMax;
        setLimelightNTDoubleArray(limelightName, "crop", entries);
    }

    /**
     * Sets the camera pose in robot space for the specified Limelight.
     * 
     * @param limelightName The name of the Limelight.
     * @param forward       The forward position of the camera in robot space.
     * @param side          The side position of the camera in robot space.
     * @param up            The up position of the camera in robot space.
     * @param roll          The roll angle of the camera in robot space.
     * @param pitch         The pitch angle of the camera in robot space.
     * @param yaw           The yaw angle of the camera in robot space.
     */
    public static void setCameraPose_RobotSpace(String limelightName, double forward, double side, double up,
            double roll, double pitch, double yaw) {
        double[] entries = new double[6];
        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    /////
    /////

    /**
     * Sets the Python script data for a specific Limelight.
     * 
     * @param limelightName      The name of the Limelight.
     * @param outgoingPythonData The outgoing Python data to be set.
     */
    public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
        setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
    }

    /**
     * Retrieves the Python script data from the specified Limelight.
     * 
     * @param limelightName the name of the Limelight
     * @return an array of double values representing the Python script data
     */
    public static double[] getPythonScriptData(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "llpython");
    }

    /////
    /////

    /**
     * Asynchronously take snapshot.
     */
    /**
     * Takes a snapshot of the current Limelight camera image and saves it with the
     * specified name.
     * 
     * @param tableName    The name of the Limelight table.
     * @param snapshotName The name to assign to the snapshot.
     * @return A CompletableFuture that completes with a boolean indicating the
     *         success of taking the snapshot.
     */
    public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
        return CompletableFuture.supplyAsync(() -> {
            return SYNCH_TAKESNAPSHOT(tableName, snapshotName);
        });
    }

    /**
     * Takes a snapshot using the Limelight camera with the specified table name and
     * snapshot name.
     * 
     * @param tableName    the name of the Limelight table
     * @param snapshotName the name of the snapshot (optional)
     * @return true if the snapshot was taken successfully, false otherwise
     */
    private static boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName) {
        URL url = getLimelightURLString(tableName, "capturesnapshot");
        try {
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            if (snapshotName != null && snapshotName != "") {
                connection.setRequestProperty("snapname", snapshotName);
            }

            int responseCode = connection.getResponseCode();
            if (responseCode == 200) {
                return true;
            } else {
                System.err.println("Bad LL Request");
            }
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
        return false;
    }

    /**
     * Parses Limelight's JSON results dump into a LimelightResults Object
     */
    /**
     * Represents the results obtained from the Limelight camera.
     */
    public static LimelightResults getLatestResults(String limelightName) {

        long start = System.nanoTime();
        LimelightHelpers.LimelightResults results = new LimelightHelpers.LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            results = mapper.readValue(getJSONDump(limelightName), LimelightResults.class);
        } catch (JsonProcessingException e) {
            System.err.println("lljson error: " + e.getMessage());
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.targetingResults.latency_jsonParse = millis;
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", millis);
        }

        return results;
    }
}