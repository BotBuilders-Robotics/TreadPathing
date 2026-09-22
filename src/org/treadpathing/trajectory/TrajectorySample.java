package org.treadpathing.trajectory;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/**
 * One instant of a generated trajectory.
 *
 * <p>Everything here is expressed in the <b>robot's</b> frame of reference rather than the
 * path's, which matters for reversed segments:
 *
 * <ul>
 *   <li>{@code pose.heading} is where the robot's nose points, so a reversed segment's
 *       heading is the path tangent plus pi.
 *   <li>{@code velocity} is signed forward speed, so it is negative when reversing.
 *   <li>{@code curvature} is {@code omega / velocity}, so it is the negated path curvature
 *       when reversing. Feeding it straight into {@code omega = v * k} then works for both
 *       directions with no special cases in the controller.
 * </ul>
 */
public final class TrajectorySample {

    private final double time;
    private final double arcLength;
    private final Pose pose;
    private final double velocity;
    private final double acceleration;
    private final double curvature;

    public TrajectorySample(double time, double arcLength, Pose pose,
                            double velocity, double acceleration, double curvature) {
        this.time = time;
        this.arcLength = arcLength;
        this.pose = pose;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.curvature = curvature;
    }

    public double getTime() {
        return time;
    }

    public double getArcLength() {
        return arcLength;
    }

    public Pose getPose() {
        return pose;
    }

    /** Signed forward speed, inches per second. Negative while reversing. */
    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }

    /** Robot-frame curvature, inverse inches. {@code omega = velocity * curvature}. */
    public double getCurvature() {
        return curvature;
    }

    /** Reference angular velocity, radians per second. */
    public double getAngularVelocity() {
        return velocity * curvature;
    }

    public TrajectorySample interpolate(TrajectorySample other, double t) {
        return new TrajectorySample(
                MathUtil.lerp(time, other.time, t),
                MathUtil.lerp(arcLength, other.arcLength, t),
                pose.interpolate(other.pose, t),
                MathUtil.lerp(velocity, other.velocity, t),
                MathUtil.lerp(acceleration, other.acceleration, t),
                MathUtil.lerp(curvature, other.curvature, t));
    }

    @Override
    public String toString() {
        return String.format("t=%.3f s=%.2f %s v=%.2f a=%.2f k=%.4f",
                time, arcLength, pose, velocity, acceleration, curvature);
    }
}
