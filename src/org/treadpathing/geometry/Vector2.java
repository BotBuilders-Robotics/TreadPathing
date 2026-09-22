package org.treadpathing.geometry;

/** Immutable 2D vector in field inches. */
public final class Vector2 {

    public static final Vector2 ZERO = new Vector2(0.0, 0.0);

    private final double x;
    private final double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2 fromPolar(double magnitude, double angleRadians) {
        return new Vector2(magnitude * Math.cos(angleRadians), magnitude * Math.sin(angleRadians));
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double norm() {
        return Math.sqrt(x * x + y * y);
    }

    public double normSquared() {
        return x * x + y * y;
    }

    /** Direction of the vector in radians, CCW from +x. */
    public double angle() {
        return Math.atan2(y, x);
    }

    public Vector2 plus(Vector2 other) {
        return new Vector2(x + other.x, y + other.y);
    }

    public Vector2 minus(Vector2 other) {
        return new Vector2(x - other.x, y - other.y);
    }

    public Vector2 times(double scalar) {
        return new Vector2(x * scalar, y * scalar);
    }

    public Vector2 div(double scalar) {
        return new Vector2(x / scalar, y / scalar);
    }

    public Vector2 unaryMinus() {
        return new Vector2(-x, -y);
    }

    public double dot(Vector2 other) {
        return x * other.x + y * other.y;
    }

    /** 2D cross product magnitude (the z component of the 3D cross product). */
    public double cross(Vector2 other) {
        return x * other.y - y * other.x;
    }

    public Vector2 rotate(double angleRadians) {
        double c = Math.cos(angleRadians);
        double s = Math.sin(angleRadians);
        return new Vector2(x * c - y * s, x * s + y * c);
    }

    public Vector2 normalized() {
        double n = norm();
        if (n < MathUtil.EPSILON) {
            return ZERO;
        }
        return new Vector2(x / n, y / n);
    }

    public double distanceTo(Vector2 other) {
        return minus(other).norm();
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f)", x, y);
    }
}
