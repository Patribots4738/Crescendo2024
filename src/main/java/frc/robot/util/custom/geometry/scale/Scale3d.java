package frc.robot.util.custom.geometry.scale;

/**
 * This class is used to represent a 3D scale.
 * Primarily, for FMAPs with april tags.
 */
public class Scale3d {
    public double x;
    public double y;
    public double z;

    public Scale3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Scale3d(double x, double y) {
        this(x, y, 1);
    }

    public Scale3d() {
        this(1, 1, 1);
    }

    public Scale3d(Scale3d scale) {
        this(scale.x, scale.y, scale.z);
    }

    public Scale3d of(double x, double y, double z) {
        return new Scale3d(x, y, z);
    }
    
    @Override
    public Scale3d of(double x, double y) {
        return new Scale3d(x, y);
    }
    @Override
    public Scale3d of() {
        return new Scale3d();
    }
    @Override
    public Scale3d of(Scale3d scale) {
        return new Scale3d(scale);
    }
    @Override
    public Scale3d add(Scale3d scale) {
        return new Scale3d(x + scale.x, y + scale.y, z + scale.z);
    }
    @Override
    public Scale3d subtract(Scale3d scale) {
        return new Scale3d(x - scale.x, y - scale.y, z - scale.z);
    }
    @Override
    public Scale3d multiply(Scale3d scale) {
        return new Scale3d(x * scale.x, y * scale.y, z * scale.z);
    }
    @Override
    public Scale3d divide(Scale3d scale) {
        return new Scale3d(x / scale.x, y / scale.y, z / scale.z);
    }
    @Override
    public Scale3d negate() {
        return new Scale3d(-x, -y, -z);
    }
}
