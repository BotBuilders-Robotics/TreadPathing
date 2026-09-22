package org.treadpathing.trajectory;

import org.treadpathing.geometry.MathUtil;

/**
 * The envelope the trajectory generator must stay inside.
 *
 * <p>Three of these limits are the ones that actually shape an FTC tank auto:
 *
 * <ul>
 *   <li><b>Centripetal</b> — {@code v <= sqrt(a_c / |k|)}. This is the traction and tip limit.
 *       Exceed it and the robot understeers out of the corner or rolls.
 *   <li><b>Wheel speed</b> — {@code v <= v_wheel / (1 + |k| * T / 2)}. The outer wheel of a
 *       turning differential drive travels further than the robot's centre, so a tight corner
 *       costs chassis speed even when traction is fine. At k = 0.1 /in with a 14 in track,
 *       that is a 41% cut.
 *   <li><b>Turning acceleration</b> — the same geometry applies to acceleration, so the
 *       usable chassis acceleration falls in a corner too.
 * </ul>
 *
 * <p>All values are inches, seconds and radians. Zero or negative disables a limit.
 */
public final class TrajectoryConstraints {

    private double maxVelocity = 40.0;
    private double maxAcceleration = 40.0;
    private double maxDeceleration = 40.0;
    private double maxCentripetalAcceleration = 50.0;
    private double maxWheelVelocity = 0.0;
    private double trackWidth = 14.0;
    private double startVelocity = 0.0;
    private double endVelocity = 0.0;

    public TrajectoryConstraints() {
    }

    public TrajectoryConstraints copy() {
        TrajectoryConstraints c = new TrajectoryConstraints();
        c.maxVelocity = maxVelocity;
        c.maxAcceleration = maxAcceleration;
        c.maxDeceleration = maxDeceleration;
        c.maxCentripetalAcceleration = maxCentripetalAcceleration;
        c.maxWheelVelocity = maxWheelVelocity;
        c.trackWidth = trackWidth;
        c.startVelocity = startVelocity;
        c.endVelocity = endVelocity;
        return c;
    }

    public TrajectoryConstraints maxVelocity(double inchesPerSecond) {
        this.maxVelocity = inchesPerSecond;
        return this;
    }

    public TrajectoryConstraints maxAcceleration(double inchesPerSecondSquared) {
        this.maxAcceleration = inchesPerSecondSquared;
        return this;
    }

    public TrajectoryConstraints maxDeceleration(double inchesPerSecondSquared) {
        this.maxDeceleration = inchesPerSecondSquared;
        return this;
    }

    public TrajectoryConstraints maxCentripetalAcceleration(double inchesPerSecondSquared) {
        this.maxCentripetalAcceleration = inchesPerSecondSquared;
        return this;
    }

    /** Peak speed a single side of the drivetrain can hold, in inches per second. */
    public TrajectoryConstraints maxWheelVelocity(double inchesPerSecond) {
        this.maxWheelVelocity = inchesPerSecond;
        return this;
    }

    /** Effective (not measured) track width. Skid steer scrub makes this 2-4 in wider. */
    public TrajectoryConstraints trackWidth(double inches) {
        this.trackWidth = inches;
        return this;
    }

    public TrajectoryConstraints startVelocity(double inchesPerSecond) {
        this.startVelocity = inchesPerSecond;
        return this;
    }

    public TrajectoryConstraints endVelocity(double inchesPerSecond) {
        this.endVelocity = inchesPerSecond;
        return this;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxDeceleration() {
        return maxDeceleration;
    }

    public double getMaxCentripetalAcceleration() {
        return maxCentripetalAcceleration;
    }

    public double getMaxWheelVelocity() {
        return maxWheelVelocity;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public double getStartVelocity() {
        return startVelocity;
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    /** Largest speed magnitude allowed at the given curvature. */
    public double velocityLimit(double curvature) {
        double k = Math.abs(curvature);
        double limit = maxVelocity;

        if (maxCentripetalAcceleration > 0.0 && k > MathUtil.EPSILON) {
            limit = Math.min(limit, Math.sqrt(maxCentripetalAcceleration / k));
        }
        if (maxWheelVelocity > 0.0) {
            limit = Math.min(limit, maxWheelVelocity / (1.0 + k * trackWidth / 2.0));
        }
        return Math.max(limit, 0.0);
    }

    /** Largest acceleration magnitude usable at the given curvature. */
    public double accelerationLimit(double curvature) {
        return maxAcceleration / (1.0 + Math.abs(curvature) * trackWidth / 2.0);
    }

    /** Largest deceleration magnitude usable at the given curvature. */
    public double decelerationLimit(double curvature) {
        return maxDeceleration / (1.0 + Math.abs(curvature) * trackWidth / 2.0);
    }

    public void validate() {
        if (maxVelocity <= 0.0) {
            throw new IllegalStateException("maxVelocity must be positive");
        }
        if (maxAcceleration <= 0.0) {
            throw new IllegalStateException("maxAcceleration must be positive");
        }
        if (maxDeceleration <= 0.0) {
            throw new IllegalStateException("maxDeceleration must be positive");
        }
        if (trackWidth <= 0.0) {
            throw new IllegalStateException("trackWidth must be positive");
        }
    }
}
