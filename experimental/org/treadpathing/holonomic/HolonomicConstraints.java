package org.treadpathing.holonomic;

import org.treadpathing.geometry.MathUtil;

/**
 * The speed envelope of a mecanum drive, which is not the tank envelope with a field added.
 *
 * <p>On a tank robot the wheel-speed limit depends on one thing: how sharply the path bends,
 * because turning is the only thing that makes one wheel outrun the other. On a mecanum three
 * separate demands compete for the same wheel budget:
 *
 * <ul>
 *   <li>going forward, which costs one unit of wheel speed per unit of travel;
 *   <li>going sideways, which costs {@code lateralMultiplier} of it, because the rollers
 *       scrub;
 *   <li>rotating, which costs {@code omega * turnRadius} whether or not the robot is moving.
 * </ul>
 *
 * <p>So the fastest the robot may travel depends on the angle between where it is going and
 * where it is pointing -- a quantity a differential drive does not have, because for it that
 * angle is always zero.
 */
public final class HolonomicConstraints {

    private double maxVelocity = 50.0;
    private double maxAcceleration = 50.0;
    private double maxDeceleration = 45.0;
    private double maxCentripetalAcceleration = 40.0;
    private double maxWheelVelocity = 60.0;
    private double maxAngularVelocity = 3.0;
    private double startVelocity;
    private double endVelocity;

    public HolonomicConstraints maxVelocity(double inchesPerSecond) {
        this.maxVelocity = inchesPerSecond;
        return this;
    }

    public HolonomicConstraints maxAcceleration(double inchesPerSecondSquared) {
        this.maxAcceleration = inchesPerSecondSquared;
        return this;
    }

    public HolonomicConstraints maxDeceleration(double inchesPerSecondSquared) {
        this.maxDeceleration = inchesPerSecondSquared;
        return this;
    }

    public HolonomicConstraints maxCentripetalAcceleration(double inchesPerSecondSquared) {
        this.maxCentripetalAcceleration = inchesPerSecondSquared;
        return this;
    }

    public HolonomicConstraints maxWheelVelocity(double inchesPerSecond) {
        this.maxWheelVelocity = inchesPerSecond;
        return this;
    }

    /** Ceiling on the turn rate itself, whatever the wheels could manage. */
    public HolonomicConstraints maxAngularVelocity(double radiansPerSecond) {
        this.maxAngularVelocity = radiansPerSecond;
        return this;
    }

    public HolonomicConstraints startVelocity(double inchesPerSecond) {
        this.startVelocity = inchesPerSecond;
        return this;
    }

    public HolonomicConstraints endVelocity(double inchesPerSecond) {
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

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public double getStartVelocity() {
        return startVelocity;
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    /**
     * Largest speed along the path that the wheels can actually deliver here.
     *
     * <p>Solved rather than iterated. Travelling at speed {@code v} along a path that bends at
     * {@code curvature}, with the nose {@code travelAngle} off the direction of travel and a
     * heading plan turning at {@code headingRate} radians per inch, the worst wheel is asked
     * for
     *
     * <pre>
     * v * (|cos travelAngle| + lateral * |sin travelAngle|) + v * |headingRate| * turnRadius
     * </pre>
     *
     * <p>which is linear in {@code v}, so the feasible {@code v} falls straight out. That is
     * why the heading plan is expressed per inch and not per second: per second the two
     * unknowns chase each other.
     *
     * @param travelAngle direction of travel in the robot's frame: 0 is straight ahead, pi/2
     *                    is straight out of the left side
     * @param headingRate radians of nose rotation per inch travelled
     */
    public double velocityLimit(double curvature, double travelAngle, double headingRate,
                                MecanumKinematics kinematics) {
        double limit = maxVelocity;
        double k = Math.abs(curvature);

        // Traction and tipping do not care which way the robot is pointing while it corners.
        if (maxCentripetalAcceleration > 0.0 && k > MathUtil.EPSILON) {
            limit = Math.min(limit, Math.sqrt(maxCentripetalAcceleration / k));
        }

        if (maxWheelVelocity > 0.0) {
            double cost = wheelCostPerInch(travelAngle, headingRate, kinematics);
            if (cost > MathUtil.EPSILON) {
                limit = Math.min(limit, maxWheelVelocity / cost);
            }
        }

        // A heading plan that turns quickly over a short path can outrun the turn rate the
        // robot has, whatever the wheels say.
        double rate = Math.abs(headingRate);
        if (maxAngularVelocity > 0.0 && rate > MathUtil.EPSILON) {
            limit = Math.min(limit, maxAngularVelocity / rate);
        }

        return Math.max(limit, 0.0);
    }

    /** Wheel speed the worst wheel needs per inch per second of travel along the path. */
    public double wheelCostPerInch(double travelAngle, double headingRate,
                                   MecanumKinematics kinematics) {
        double forward = Math.abs(Math.cos(travelAngle));
        double lateral = Math.abs(Math.sin(travelAngle)) * kinematics.getLateralMultiplier();
        double rotation = Math.abs(headingRate) * kinematics.getTurnRadius();
        return forward + lateral + rotation;
    }

    /**
     * Acceleration is derated by the same wheel cost as velocity: a robot strafing at 1.15
     * units of wheel speed per unit of travel cannot accelerate sideways as hard as it
     * accelerates forwards.
     */
    public double accelerationLimit(double travelAngle, double headingRate,
                                    MecanumKinematics kinematics) {
        return maxAcceleration / Math.max(wheelCostPerInch(travelAngle, headingRate, kinematics), 1.0);
    }

    public double decelerationLimit(double travelAngle, double headingRate,
                                    MecanumKinematics kinematics) {
        return maxDeceleration / Math.max(wheelCostPerInch(travelAngle, headingRate, kinematics), 1.0);
    }

    public HolonomicConstraints copy() {
        return new HolonomicConstraints()
                .maxVelocity(maxVelocity)
                .maxAcceleration(maxAcceleration)
                .maxDeceleration(maxDeceleration)
                .maxCentripetalAcceleration(maxCentripetalAcceleration)
                .maxWheelVelocity(maxWheelVelocity)
                .maxAngularVelocity(maxAngularVelocity)
                .startVelocity(startVelocity)
                .endVelocity(endVelocity);
    }

    public void validate() {
        if (maxVelocity <= 0.0) {
            throw new IllegalStateException("maxVelocity must be positive");
        }
        if (maxAcceleration <= 0.0 || maxDeceleration <= 0.0) {
            throw new IllegalStateException("maxAcceleration and maxDeceleration must be positive");
        }
    }
}
