package org.treadpathing.geometry;

/**
 * Small numeric helpers shared across the library.
 *
 * <p>Everything in Tread Pathing uses <b>inches, radians and seconds</b>. Headings are
 * measured counter-clockwise from the +x axis, matching the Pedro Pathing convention
 * (corner origin, field spans 0..144 inches in both axes).
 *
 * <p>Java 7 only: no lambdas, no streams, no java.util.function.
 */
public final class MathUtil {

    /** Field size in inches. The origin is the bottom-left corner. */
    public static final double FIELD_SIZE = 144.0;

    public static final double EPSILON = 1e-9;
    public static final double TAU = 2.0 * Math.PI;

    private MathUtil() {
    }

    public static double clamp(double value, double low, double high) {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    public static int clamp(int value, int low, int high) {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    /** Wraps an angle into (-pi, pi]. */
    public static double normalizeAngle(double radians) {
        double a = radians % TAU;
        if (a > Math.PI) {
            a -= TAU;
        } else if (a <= -Math.PI) {
            a += TAU;
        }
        return a;
    }

    /** Shortest signed rotation that takes {@code from} to {@code to}, in (-pi, pi]. */
    public static double angleDelta(double from, double to) {
        return normalizeAngle(to - from);
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    /** Interpolates between two angles along the shortest arc. */
    public static double lerpAngle(double a, double b, double t) {
        return normalizeAngle(a + angleDelta(a, b) * t);
    }

    /**
     * sin(x)/x, with a Taylor expansion near zero. Ramsete evaluates this exactly when the
     * heading error is zero, so the naive form would return NaN at the moment the robot is
     * perfectly aligned.
     */
    public static double sinc(double x) {
        if (Math.abs(x) < 1e-6) {
            return 1.0 - x * x / 6.0;
        }
        return Math.sin(x) / x;
    }

    public static boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < 1e-6;
    }

    public static boolean epsilonEquals(double a, double b, double tolerance) {
        return Math.abs(a - b) < tolerance;
    }

    /** signum that returns +1 for zero, so it never zeroes out a feedforward term. */
    public static double signumNonZero(double x) {
        return x < 0.0 ? -1.0 : 1.0;
    }

    public static double hypot(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Solves {@code v_f = sqrt(v_i^2 + 2*a*d)} while keeping the result real. Used by both
     * passes of the trajectory parameterizer.
     */
    public static double reachableVelocity(double initialVelocity, double acceleration, double distance) {
        double squared = initialVelocity * initialVelocity + 2.0 * acceleration * distance;
        if (squared <= 0.0) {
            return 0.0;
        }
        return Math.sqrt(squared);
    }

    /** Converts degrees to radians. Provided so team code never has to import anything else. */
    public static double toRadians(double degrees) {
        return degrees * Math.PI / 180.0;
    }

    public static double toDegrees(double radians) {
        return radians * 180.0 / Math.PI;
    }
}
