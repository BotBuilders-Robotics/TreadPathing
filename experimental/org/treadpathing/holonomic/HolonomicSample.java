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

    public HolonomicSample(double time, double arcLength, Pose pose,
                           double fieldVx, double fieldVy, double omega, double acceleration) {
        this.time = time;
        this.arcLength = arcLength;
        this.pose = pose;
        this.fieldVx = fieldVx;
        this.fieldVy = fieldVy;
        this.omega = omega;
        this.acceleration = acceleration;
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

    /** Acceleration along the path, inches per second squared. */
    public double getAcceleration() {
        return acceleration;
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
