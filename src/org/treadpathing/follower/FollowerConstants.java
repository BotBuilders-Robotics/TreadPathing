package org.treadpathing.follower;

import org.treadpathing.control.LtvUnicycleController;
import org.treadpathing.control.Pid;
import org.treadpathing.control.PoseHoldController;
import org.treadpathing.control.RamseteController;
import org.treadpathing.control.TrajectoryController;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.hardware.DriveConstants;
import org.treadpathing.route.RouteDefaults;

/**
 * Everything the follower needs that is not hardware: which controller, how hard to push,
 * and how precisely to stop.
 *
 * <p>The defaults here are starting points that will drive a route on a typical FTC tank
 * robot. They are not a substitute for the tuning ladder — a team that skips it will get an
 * auto that works and misses by two inches.
 */
public final class FollowerConstants {

    /** Which feedback law tracks the trajectory. */
    public enum Controller {
        /**
         * The default. Its gain is a closed form costing one square root per loop, and it is
         * deprecated in WPILib rather than broken.
         */
        RAMSETE,
        /**
         * LQR-optimal gains from a precomputed table. Better tracking, and tuned by stating
         * error tolerances rather than picking a number in rad^2/in^2 — but the table has to
         * be regenerated offline if your loop period is far from 50 Hz.
         */
        LTV
    }

    private final DriveConstants driveConstants;
    private RouteDefaults routeDefaults = new RouteDefaults();

    private Controller controller = Controller.RAMSETE;
    private double ramseteB = RamseteController.DEFAULT_B_PER_INCH_SQUARED;
    private double ramseteZeta = RamseteController.DEFAULT_ZETA;
    private double ltvGainScale = 1.0;

    private double holdDistanceGain = 2.5;
    private double holdBearingGain = 2.2;
    private double holdHeadingGain = 2.2;
    private double holdHeadingDamping = 0.05;
    private double positionTolerance = 0.5;
    private double headingTolerance = MathUtil.toRadians(2.0);
    private double holdMaxLinear = 18.0;
    private double holdMaxAngular = 2.5;

    private double minLoopPeriod = 0.005;
    private double maxLoopPeriod = 0.100;

    public FollowerConstants(DriveConstants driveConstants) {
        this.driveConstants = driveConstants;
        // Keep the route's motion envelope consistent with the drivetrain by default, so a
        // team that only fills in DriveConstants still gets feasible trajectories.
        this.routeDefaults.getConstraints()
                .trackWidth(driveConstants.getTrackWidth())
                .maxWheelVelocity(driveConstants.getMaxWheelVelocity())
                .maxVelocity(driveConstants.getMaxWheelVelocity() * 0.85);
    }

    public DriveConstants getDriveConstants() {
        return driveConstants;
    }

    public RouteDefaults getRouteDefaults() {
        return routeDefaults;
    }

    public FollowerConstants routeDefaults(RouteDefaults defaults) {
        this.routeDefaults = defaults;
        return this;
    }

    public FollowerConstants controller(Controller value) {
        this.controller = value;
        return this;
    }

    /** Ramsete aggressiveness, in rad^2/in^2. See {@link RamseteController} on units. */
    public FollowerConstants ramsete(double bPerInchSquared, double zeta) {
        this.ramseteB = bPerInchSquared;
        this.ramseteZeta = zeta;
        return this;
    }

    public FollowerConstants ltvGainScale(double scale) {
        this.ltvGainScale = scale;
        return this;
    }

    public FollowerConstants poseHoldGains(double distance, double bearing, double heading) {
        this.holdDistanceGain = distance;
        this.holdBearingGain = bearing;
        this.holdHeadingGain = heading;
        return this;
    }

    /** How close counts as arrived. Below about a third of an inch, odometry noise wins. */
    public FollowerConstants tolerances(double positionInches, double headingRadians) {
        this.positionTolerance = positionInches;
        this.headingTolerance = headingRadians;
        return this;
    }

    public FollowerConstants poseHoldLimits(double maxLinear, double maxAngular) {
        this.holdMaxLinear = maxLinear;
        this.holdMaxAngular = maxAngular;
        return this;
    }

    /**
     * Bounds on the measured loop period. Clamping matters: a garbage collection pause of a
     * third of a second must degrade one control step, not corrupt the pose estimate by
     * integrating as though the robot really moved for that long.
     */
    public FollowerConstants loopPeriodBounds(double minSeconds, double maxSeconds) {
        this.minLoopPeriod = minSeconds;
        this.maxLoopPeriod = maxSeconds;
        return this;
    }

    public double getMinLoopPeriod() {
        return minLoopPeriod;
    }

    public double getMaxLoopPeriod() {
        return maxLoopPeriod;
    }

    public double getPositionTolerance() {
        return positionTolerance;
    }

    public double getHeadingTolerance() {
        return headingTolerance;
    }

    public TrajectoryController buildTrajectoryController() {
        if (controller == Controller.LTV) {
            return new LtvUnicycleController(ltvGainScale);
        }
        return new RamseteController(ramseteB, ramseteZeta);
    }

    public PoseHoldController buildPoseHoldController() {
        return new PoseHoldController(
                new Pid(holdDistanceGain, 0.0, 0.0),
                new Pid(holdBearingGain, 0.0, holdHeadingDamping, 0.0, 0.6),
                new Pid(holdHeadingGain, 0.0, holdHeadingDamping, 0.0, 0.6),
                positionTolerance, headingTolerance, holdMaxLinear, holdMaxAngular);
    }
}
