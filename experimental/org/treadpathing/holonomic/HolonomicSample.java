package org.treadpathing.holonomic;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/**
 * One instant of a holonomic trajectory.
 *
 * <p>Where the tank sample carries a signed forward speed and a curvature -- two numbers that
 * only describe motion because heading and travel are the same thing there -- this carries a
 * field-frame velocity vector and a heading rate that has nothing to do with it.
 */
public final class HolonomicSample {

    private final double time;
    private final double arcLength;
    private final Pose pose;
    private final double fieldVx;
    private final double fieldVy;
    private final double omega;
    private final double acceleration;
    private final double fieldAx;
    private final double fieldAy;
    private final double alpha;

    /**
     * @param acceleration rate of change of speed along the path, inches per second squared
     * @param fieldAx      field-frame acceleration, both along the path and into the bend
     * @param fieldAy      field-frame acceleration, both along the path and into the bend
     * @param alpha        angular acceleration, radians per second squared
     */
    public HolonomicSample(double time, double arcLength, Pose pose,
                           double fieldVx, double fieldVy, double omega, double acceleration,
                           double fieldAx, double fieldAy, double alpha) {
        this.time = time;
        this.arcLength = arcLength;
        this.pose = pose;
        this.fieldVx = fieldVx;
        this.fieldVy = fieldVy;
        this.omega = omega;
        this.acceleration = acceleration;
        this.fieldAx = fieldAx;
        this.fieldAy = fieldAy;
        this.alpha = alpha;
    }

    public double getTime() {
        return time;
    }

    public double getArcLength() {
        return arcLength;
    }

    /** Where the robot should be, and where its nose should point. */
    public Pose getPose() {
        return pose;
    }

    public double getFieldVx() {
        return fieldVx;
    }

    public double getFieldVy() {
        return fieldVy;
    }

    public double getOmega() {
        return omega;
    }

    /** Speed along the path, always positive: a holonomic path is never driven backwards. */
    public double getSpeed() {
        return Math.hypot(fieldVx, fieldVy);
    }

    /**
     * Acceleration along the path, inches per second squared.
     *
     * <p>A scalar, so it says nothing about which way any wheel is turning. Feed the motors
     * {@link #robotAcceleration} instead.
     */
    public double getAcceleration() {
        return acceleration;
    }

    public double getFieldAx() {
        return fieldAx;
    }

    public double getFieldAy() {
        return fieldAy;
    }

    public double getAlpha() {
        return alpha;
    }

    /**
     * The acceleration each wheel has to find, as a robot-frame command: pass it through
     * {@link HolonomicSpeeds#wheelSpeeds} and the result is per-wheel acceleration.
     *
     * <p>The tank drive can hand one number to both sides because both sides always speed up
     * together. A mecanum cannot: in a strafe left the front-left and back-right wheels run
     * backwards, so speeding up the robot means speeding those wheels up <b>in reverse</b>,
     * and the same kA term applied with the path's sign pushes them the wrong way.
     *
     * <p>The {@code omega} terms are there because the robot frame itself turns: a robot
     * holding a steady field velocity while its nose swings round still has wheels that
     * speed up and slow down.
     *
     * @param heading the heading the command is being rotated by, which is the measured one
     */
    public HolonomicSpeeds robotAcceleration(double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double ax = fieldAx * cos + fieldAy * sin;
        double ay = -fieldAx * sin + fieldAy * cos;
        double vx = fieldVx * cos + fieldVy * sin;
        double vy = -fieldVx * sin + fieldVy * cos;
        return new HolonomicSpeeds(ax + omega * vy, ay - omega * vx, alpha);
    }

    /**
     * Direction of travel in the robot's frame: 0 is straight ahead, pi/2 straight out of the
     * left side. Always 0 on a tank drive, by definition -- which is the whole point.
     *
     * <p>A stopped robot is not travelling in any direction, so the ends of a trajectory,
     * where reference velocity is zero, report 0 rather than an angle read out of atan2(0, 0).
     */
    public double getTravelAngle() {
        if (getSpeed() < MathUtil.EPSILON) {
            return 0.0;
        }
        return MathUtil.normalizeAngle(Math.atan2(fieldVy, fieldVx) - pose.getHeading());
    }
}
