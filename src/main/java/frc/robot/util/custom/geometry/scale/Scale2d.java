package frc.robot.util.custom.geometry.scale;

public class Scale2d implements Scale<Scale2d>{
    public double x;
    public double y;

    public Scale2d(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    public Scale2d() {
        this(1, 1);
    }

    public Scale2d(Scale2d scale) {
        this(scale.x, scale.y);
    }


    @Override
    public Scale2d add(Scale2d scale) {
        return new Scale2d(x + scale.x, y + scale.y);
    }

    @Override
    public Scale2d subtract(Scale2d scale) {
        return new Scale2d(x - scale.x, y - scale.y);
    }

    @Override
    public Scale2d multiply(Scale2d scale) {
        return new Scale2d(x * scale.x, y * scale.y);
    }

    @Override
    public Scale2d divide(Scale2d scale) {
        return new Scale2d(x / scale.x, y / scale.y);
    }

    @Override
    public Scale2d negate() {
        return new Scale2d(-x, -y);
    }

    @Override
    public Scale2d of(double x, double y) {
        return new Scale2d(x, y);
    }

    @Override
    public Scale2d of() {
        return new Scale2d();
    }

    @Override
    public Scale2d of(Scale2d scale) {
        return new Scale2d(scale);
    }
    
}
