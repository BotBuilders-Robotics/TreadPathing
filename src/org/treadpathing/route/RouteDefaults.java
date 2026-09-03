package org.treadpathing.route;

import org.treadpathing.control.Pid;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.trajectory.TrajectoryConstraints;

/**
 * Everything {@link RouteBuilder} needs that is not part of the route itself: default motion
 * limits, turn gains and tolerances.
 *
 * <p>A team fills one of these in once, in their {@code Constants} class, and never thinks
 * about it again.
 */
public final class RouteDefaults {

    private TrajectoryConstraints constraints = new TrajectoryConstraints();

    private double maxAngularVelocity = 3.0;
    private double maxAngularAcceleration = 6.0;
    private double turnKP = 2.0;
    private double turnKD = 0.08;
    private double headingTolerance = MathUtil.toRadians(2.0);
    private double angularVelocityTolerance = MathUtil.toRadians(8.0);
    private double turnSettleTime = 0.10;
    private double turnTimeout = 3.0;

    private double holdTimeout = 1.5;
    private double holdSettleTime = 0.12;
    private double actionTimeout = 5.0;

    /** Heading mismatch above which {@code lineTo} inserts a turn before driving. */
    private double lineHeadingTolerance = MathUtil.toRadians(2.0);

    public TrajectoryConstraints getConstraints() {
        return constraints;
    }

    public RouteDefaults constraints(TrajectoryConstraints constraints) {
        this.constraints = constraints;
        return this;
    }

    public RouteDefaults maxAngularVelocity(double radiansPerSecond) {
        this.maxAngularVelocity = radiansPerSecond;
        return this;
    }

    public RouteDefaults maxAngularAcceleration(double radiansPerSecondSquared) {
        this.maxAngularAcceleration = radiansPerSecondSquared;
        return this;
    }

    public RouteDefaults turnGains(double kP, double kD) {
        this.turnKP = kP;
        this.turnKD = kD;
        return this;
    }

    public RouteDefaults headingTolerance(double radians) {
        this.headingTolerance = radians;
        return this;
    }

    public RouteDefaults angularVelocityTolerance(double radiansPerSecond) {
        this.angularVelocityTolerance = radiansPerSecond;
        return this;
    }

    public RouteDefaults turnSettleTime(double seconds) {
        this.turnSettleTime = seconds;
        return this;
    }

    public RouteDefaults turnTimeout(double seconds) {
        this.turnTimeout = seconds;
        return this;
    }

    public RouteDefaults holdTimeout(double seconds) {
        this.holdTimeout = seconds;
        return this;
    }

    public RouteDefaults holdSettleTime(double seconds) {
        this.holdSettleTime = seconds;
        return this;
    }

    public RouteDefaults actionTimeout(double seconds) {
        this.actionTimeout = seconds;
        return this;
    }

    public RouteDefaults lineHeadingTolerance(double radians) {
        this.lineHeadingTolerance = radians;
        return this;
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public double getMaxAngularAcceleration() {
        return maxAngularAcceleration;
    }

    public double getHeadingTolerance() {
        return headingTolerance;
    }

    public double getAngularVelocityTolerance() {
        return angularVelocityTolerance;
    }

    public double getTurnSettleTime() {
        return turnSettleTime;
    }

    public double getTurnTimeout() {
        return turnTimeout;
    }

    public double getHoldTimeout() {
        return holdTimeout;
    }

    public double getHoldSettleTime() {
        return holdSettleTime;
    }

    public double getActionTimeout() {
        return actionTimeout;
    }

    public double getLineHeadingTolerance() {
        return lineHeadingTolerance;
    }

    /** Fresh PID for a turn segment; each segment gets its own so state never leaks. */
    public Pid newTurnPid() {
        return new Pid(turnKP, 0.0, turnKD, 0.0, 0.6);
    }
}
