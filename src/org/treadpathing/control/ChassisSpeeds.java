package org.treadpathing.control;

/**
 * A differential drive's complete command space: forward speed and turn rate.
 *
 * <p>There is no lateral term, and that absence is the whole reason this library exists
 * rather than being a fork of a mecanum follower.
 */
public final class ChassisSpeeds {

    public static final ChassisSpeeds ZERO = new ChassisSpeeds(0.0, 0.0);

    private final double linear;
    private final double angular;

    /**
     * @param linear  forward speed, inches per second, negative when reversing
     * @param angular turn rate, radians per second, positive counter-clockwise
     */
    public ChassisSpeeds(double linear, double angular) {
        this.linear = linear;
        this.angular = angular;
    }

    public double getLinear() {
        return linear;
    }

    public double getAngular() {
        return angular;
    }

    /** Left wheel speed for a given effective track width, inches per second. */
    public double leftWheelSpeed(double trackWidth) {
        return linear - angular * trackWidth / 2.0;
    }

    /** Right wheel speed for a given effective track width, inches per second. */
    public double rightWheelSpeed(double trackWidth) {
        return linear + angular * trackWidth / 2.0;
    }

    /**
     * Scales both terms down together if either wheel would be asked for more than
     * {@code maxWheelSpeed}. Scaling both preserves the commanded arc; clipping only the
     * faster wheel would silently change the path the robot drives.
     */
    public ChassisSpeeds desaturate(double trackWidth, double maxWheelSpeed) {
        if (maxWheelSpeed <= 0.0) {
            return this;
        }
        double left = leftWheelSpeed(trackWidth);
        double right = rightWheelSpeed(trackWidth);
        double peak = Math.max(Math.abs(left), Math.abs(right));
        if (peak <= maxWheelSpeed) {
            return this;
        }
        double scale = maxWheelSpeed / peak;
        return new ChassisSpeeds(linear * scale, angular * scale);
    }

    public ChassisSpeeds plus(ChassisSpeeds other) {
        return new ChassisSpeeds(linear + other.linear, angular + other.angular);
    }

    @Override
    public String toString() {
        return String.format("v=%.2f in/s, w=%.3f rad/s", linear, angular);
    }
}
