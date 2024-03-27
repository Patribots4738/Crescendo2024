package lib.limelightHelpers;

import org.json.simple.JSONObject;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N4;
import frc.robot.Constants.FieldConstants;
import lib.Scale3d;
import monologue.Logged;
import monologue.Annotations.Log;

import java.util.HashMap;

/**
 * We are turning this into the FMAP JSON format.
 * Here is a sample:
 * {
 *      "fiducials": [
 *          {
 *              "family": "apriltag3_36h11_classic",
 *              "id": 1,
 *              "size": 165.1,
 *              "transform": [
 *                  -0.5,
 *                  -0.866025,
 *                  0,
 *                  6.808597,
 *                  0.866025,
 *                  -0.5,
 *                  0,
 *                  -3.859403,
 *                  0,
 *                  0,
 *                  1,
 *                  1.355852,
 *                  0,
 *                  0,
 *                  0,
 *                  1
 *              ],
 *              "unique": 1
 *          }
 *      ]
 *  }
 */
public class LimelightConversion implements Logged {
    @Log
    public final HashMap<String, JSONObject[]> tagMap = new HashMap<>();
    private int tagLength = 16;

    /*
     * This file goes under the major-row assumption, assuming that your matrix is layed out like so:
     * r[0]  r[1]  r[2]  r[3]
     * r[4]  r[5]  r[6]  r[7]
     * r[8]  r[9]  r[10] r[11]
     * r[12] r[13] r[14] r[15]
     * 
     * this is why the math from https://bhavesh7393.artstation.com/pages/3d-transformation-to-4x4-matrix
     * is mirrored diagonally to ours, as houdini uses a major-col assumption
     * 
     * we use a right handed coordinate system and the major-row is is described here: 
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems
     * 
     */
    public LimelightConversion() {
        tagMap.put("fiducials", new JSONObject[tagLength]);
    }

    public void addAll(Pose3d[] poses) {
        for (int i = 0; i < tagMap.get("fiducials").length; i++) {
            int id = i+1;
            addFiducial(id, poses[i]);
        }
    }

    @SuppressWarnings("unchecked")
    public void addFiducial(int id, Pose3d pose3d) {
        double[] transform = (makeTransform(pose3d, CoordinateSystem.RIGHT_HANDED)).getData();
        tagMap.get("fiducials")[id-1] = new JSONObject(){
            {
                put("family", "apriltag3_36h11_classic");
                put("id", id);
                put("size", 165.1);
                put("transform", transform);
                put("unique", 1);
            }
        };
    }

    private Matrix<N4, N4> initMatrix() {
        Matrix<N4, N4> transform = new Matrix<>(Nat.N4(), Nat.N4());
        // intialize the matrix
        transform.set(0, 0, 1);
        transform.set(1, 1, 1);
        transform.set(2, 2, 1);
        transform.set(3, 3, 1);
        return transform;
    }

    public Matrix<N4, N4> makeTransform(Pose3d pose3d, CoordinateSystem dcc) {
        pose3d = getTagPoseRelativeToFMAPOrigin(pose3d);
        Translation3d translation3d = pose3d.getTranslation();
        Rotation3d rotation3d = pose3d.getRotation();
        Scale3d scale3d = new Scale3d();
        Matrix<N4, N4> transform = makeTranslationScaled(translation3d, scale3d);
        transform = transform.times(makeRotation(rotation3d, dcc));
        return transform;
    }

    public String matrixToString(Matrix<N4, N4> matrix) {
        String matrixString = "";
        for (int i = 0; i < matrix.getNumRows(); i++) {
            for (int j = 0; j < matrix.getNumCols(); j++) {
                // The weird multiplication here is removing the scientific notation
                // We are uh, totally not rounding to 8 decimal places becuase 3 isn't good enough :)
                matrixString += Math.round(matrix.get(j, i)*100000000)/100000000.0 + ((i == 3 && j == 3) ? "" : ",");
                matrixString += "\n";
            }
        }
        return matrixString;
    }

    public Pose3d getTagPoseRelativeToFMAPOrigin(Pose3d tagPose) {
        // we don't input a new pose3d w/o rotation and then apply the rotation to the relative pose
        // because then the tags would be rotated a different way in a different field space
        return tagPose.relativeTo(
                    new Pose3d(
                        new Translation3d(
                            FieldConstants.FIELD_WIDTH_METERS / 2.0, 
                            FieldConstants.FIELD_HEIGHT_METERS / 2.0, 
                            0
                        ), 
                    new Rotation3d()
                )
            );
    }

    // translation and scale matrix methods
    private Matrix<N4, N4> makeTranslationScaled(Translation3d translation3d, Scale3d scale3d) {
        Matrix<N4, N4> transform = makeTranslation(translation3d);
        transform = transform.times(makeScale(scale3d));
        return transform;
    }

    private Matrix<N4, N4> makeTranslation(Translation3d translation3d) {
        Matrix<N4, N4> transform = initMatrix();

        // init the x, y, z values
        transform.set(0, 3, translation3d.getX());
        transform.set(1, 3, translation3d.getY());
        transform.set(2, 3, translation3d.getZ());

        return transform;
    }

    private Matrix<N4, N4> makeScale(Scale3d scale3d) {
        Matrix<N4, N4> transform = initMatrix();

        // init the x, y, z values
        transform.set(0, 0, scale3d.x);
        transform.set(1, 1, scale3d.y);
        transform.set(2, 2, scale3d.z);

        return transform;
    }

    // ration matrix methods
    private double evalCos(double angle, boolean isLeftHanded) {
        return Math.cos(isLeftHanded ? angle : -angle);
    }

    private double evalSin(double angle, boolean isLeftHanded) {
        return isLeftHanded ? Math.sin(angle) : -Math.sin(angle);
    }

    private Matrix<N4, N4> makeRotation(Rotation3d rotation3d, CoordinateSystem dcc) {
        Matrix<N4, N4> rotation = initMatrix();
        // init the X rotation
        rotation = rotation.times(makeRotationX(rotation3d, dcc));

        // init the Y rotation
        rotation = rotation.times(makeRotationY(rotation3d, dcc));

        // init the Z rotation
        rotation = rotation.times(makeRotationZ(rotation3d, dcc));

        return rotation;
    }

    private Matrix<N4, N4> makeRotationX(Rotation3d rotation, CoordinateSystem dcc) {
        Matrix<N4, N4> rotationXMatrix = initMatrix();

        boolean isLeftHanded = dcc == CoordinateSystem.LEFT_HANDED;

        // init the X rotation
        rotationXMatrix.set(1, 1, evalCos(rotation.getX(), isLeftHanded));
        rotationXMatrix.set(2, 1, evalSin(rotation.getX(), isLeftHanded));
        rotationXMatrix.set(1, 2, -evalSin(rotation.getX(), isLeftHanded));
        rotationXMatrix.set(2, 2, evalCos(rotation.getX(), isLeftHanded));

        return rotationXMatrix;
    }

    private Matrix<N4, N4> makeRotationY(Rotation3d rotation, CoordinateSystem dcc) {
        Matrix<N4, N4> rotationYMatrix = initMatrix();

        boolean isLeftHanded = dcc == CoordinateSystem.LEFT_HANDED;

        // init the Y rotation
        rotationYMatrix.set(0, 0, evalCos(rotation.getY(), isLeftHanded));
        rotationYMatrix.set(2, 0, -evalSin(rotation.getY(), isLeftHanded));
        rotationYMatrix.set(0, 2, evalSin(rotation.getY(), isLeftHanded));
        rotationYMatrix.set(2, 2, evalCos(rotation.getY(), isLeftHanded));

        return rotationYMatrix;
    }

    private Matrix<N4, N4> makeRotationZ(Rotation3d rotation, CoordinateSystem dcc) {
        Matrix<N4, N4> rotationZMatrix = initMatrix();

        boolean isLeftHanded = dcc == CoordinateSystem.LEFT_HANDED;

        // init the Z rotation
        rotationZMatrix.set(0, 0, evalCos(rotation.getZ(), isLeftHanded));
        rotationZMatrix.set(1, 0, evalSin(rotation.getZ(), isLeftHanded));
        rotationZMatrix.set(0, 1, -evalSin(rotation.getZ(), isLeftHanded));
        rotationZMatrix.set(1, 1, evalCos(rotation.getZ(), isLeftHanded));

        return rotationZMatrix;
    }

    public enum CoordinateSystem {
        LEFT_HANDED, RIGHT_HANDED
    }
}
