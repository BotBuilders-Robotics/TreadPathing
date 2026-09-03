package org.treadpathing.control;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/**
 * Drives to and holds an exact pose. This is what actually sets a route's terminal accuracy.
 *
 * <p>No trajectory follower converges at the end of its own trajectory: the reference
 * velocity is zero there, and a differential drive's lateral authority is proportional to
 * velocity, so it goes to zero with it. That is a property of the vehicle, not a deficiency
 * in Ramsete or LTV. The fix is a separate primitive that is allowed to drive a small
 * correction arc.
 *
 * <p>The control law blends between two jobs rather than switching between them:
 *
 * <pre>
 *   w       = clamp((distance - tolerance) / blendRadius, 0, 1)
 *   linear  = kDist * signedDistance * max(0, cos(bearing))
 *   angular = w * kBearing * bearing + (1 - w) * kHeading * headingError
 * </pre>
 *
 * <p>Far away, {@code w} is 1: point at the target and drive to it, choosing forward or
 * reverse by whichever needs less turning, with speed faded by the cosine of the bearing so a
 * badly misaligned robot turns before it drives.
 *
 * <p>Close in, {@code w} falls to 0 and the controller stops chasing the bearing. That matters
 * more than it sounds. A robot sitting three quarters of an inch to the side of its target has
 * a bearing of nearly ninety degrees, and a controller that chases it will spin hard to fix a
 * trivial position error, wrecking the heading it had already nailed. Blending it out trades
 * the last fraction of an inch for a heading that lands — which is the right trade, because a
 * differential drive genuinely cannot fix a small lateral offset without manoeuvring.
 */
public final class PoseHoldController {

    /**
     * Inches over which the controller hands over from chasing position to holding heading.
     *
     * <p>Small on purpose. Making it large enough to matter at ordinary approach distances
     * produces a limit cycle: the robot turns toward the target, closes to within the blend
     * radius, turns back to its target heading, drifts sideways again, and repeats.
     */
    public static final double DEFAULT_BLEND_RADIUS = 1.5;

    private final Pid distancePid;
    private final Pid bearingPid;
    private final Pid headingPid;
    private final double positionTolerance;
    private final double headingTolerance;
    private final double maxLinear;
    private final double maxAngular;
    private final double blendRadius;

    public PoseHoldController(double kPDistance, double kPHeading,
                              double positionTolerance, double headingTolerance,
                              double maxLinear, double maxAngular) {
        this(new Pid(kPDistance, 0.0, 0.0),
                new Pid(kPHeading, 0.0, 0.0),
                new Pid(kPHeading, 0.0, 0.0),
                positionTolerance, headingTolerance, maxLinear, maxAngular, DEFAULT_BLEND_RADIUS);
    }

    public PoseHoldController(Pid distancePid, Pid bearingPid, Pid headingPid,
                              double positionTolerance, double headingTolerance,
                              double maxLinear, double maxAngular) {
        this(distancePid, bearingPid, headingPid, positionTolerance, headingTolerance,
                maxLinear, maxAngular, DEFAULT_BLEND_RADIUS);
    }

    public PoseHoldController(Pid distancePid, Pid bearingPid, Pid headingPid,
                              double positionTolerance, double headingTolerance,
                              double maxLinear, double maxAngular, double blendRadius) {
        this.distancePid = distancePid;
        this.bearingPid = bearingPid;
        this.headingPid = headingPid;
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
        this.maxLinear = maxLinear;
        this.maxAngular = maxAngular;
        this.blendRadius = Math.max(blendRadius, 1e-3);
    }

    public void reset() {
        distancePid.reset();
        bearingPid.reset();
        headingPid.reset();
    }

    public double getPositionTolerance() {
        return positionTolerance;
    }

    public double getHeadingTolerance() {
        return headingTolerance;
    }

    public ChassisSpeeds calculate(Pose measured, Pose target, double dt) {
        Pose error = target.relativeTo(measured);
        double distance = Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY());
        double weight = MathUtil.clamp((distance - positionTolerance) / blendRadius, 0.0, 1.0);

        double bearing = Math.atan2(error.getY(), error.getX());
        boolean reverse = Math.abs(bearing) > Math.PI / 2.0;
        if (reverse) {
            bearing = MathUtil.normalizeAngle(bearing + Math.PI);
        }
        double signedDistance = reverse ? -distance : distance;

        // Both loops run every cycle so their derivative terms stay coherent; the blend picks
        // which one is in charge rather than switching controllers on and off.
        double towardTarget = bearingPid.calculate(bearing, dt);
        double towardHeading = headingPid.calculate(error.getHeading(), dt);

        double linear = distancePid.calculate(signedDistance, dt) * Math.max(0.0, Math.cos(bearing));
        double angular = weight * towardTarget + (1.0 - weight) * towardHeading;

        return new ChassisSpeeds(
                MathUtil.clamp(linear, -maxLinear, maxLinear),
                MathUtil.clamp(angular, -maxAngular, maxAngular));
    }

    public boolean atTarget(Pose measured, Pose target) {
        return measured.distanceTo(target) < positionTolerance
                && Math.abs(MathUtil.angleDelta(measured.getHeading(), target.getHeading())) < headingTolerance;
    }
}
