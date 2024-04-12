package frc.robot.util.custom.geometry.scale;

@SuppressWarnings("hiding")
public interface Scale<Scale> {
    public Scale add(Scale scale);
    public Scale subtract(Scale scale);
    public Scale multiply(Scale scale);
    public Scale divide(Scale scale);
    public Scale negate();
    public Scale of(double x, double y);
    public Scale of();
    public Scale of(Scale scale);
}
