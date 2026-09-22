package org.treadpathing.route;

import org.treadpathing.control.ChassisSpeeds;
import org.treadpathing.control.Pid;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.trajectory.MotionProfile;

/**
 * A profiled turn in place.
 *
 * <p>A pure rotation covers zero arc length, so it cannot be expressed as a trajectory — the
 * time integration divides by distance travelled and the generator would stall. Turns get
 * their own primitive, driven by a trapezoidal heading profile plus a PID on heading error,
 * and steered entirely by the IMU.
 */
public final class TurnSegment extends Segment {

    private final double targetHeading;
    private final boolean absolute;
    private final double maxAngularVelocity;
    private final double maxAngularAcceleration;
    private final Pid headingPid;
    private final double headingTolerance;
    private final double velocityTolerance;
    private final double settleTime;
    private final double timeout;

    private MotionProfile profile;
    private double startHeading;
    private double resolvedTarget;
    private double startTime;
    private double inToleranceSince;

    /**
     * @param targetHeading          radians; absolute field heading, or a delta if
     *                               {@code absolute} is false
     * @param maxAngularVelocity     radians per second
     * @param maxAngularAcceleration radians per second squared
     * @param timeout                seconds after which the turn gives up and moves on
     */
    public TurnSegment(double targetHeading, boolean absolute,
                       double maxAngularVelocity, double maxAngularAcceleration,
                       Pid headingPid, double headingTolerance, double velocityTolerance,
                       double settleTime, double timeout) {
        this.targetHeading = targetHeading;
        this.absolute = absolute;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
        this.headingPid = headingPid;
        this.headingTolerance = headingTolerance;
        this.velocityTolerance = velocityTolerance;
        this.settleTime = settleTime;
        this.timeout = timeout;
    }

    @Override
    public void init(SegmentHost host) {
        startHeading = host.getPose().getHeading();
        double delta = absolute
                ? MathUtil.angleDelta(startHeading, targetHeading)
                : targetHeading;
        resolvedTarget = MathUtil.normalizeAngle(startHeading + delta);
        profile = new MotionProfile(delta, maxAngularVelocity, maxAngularAcceleration);
        startTime = host.time();
        inToleranceSince = -1.0;
        headingPid.reset();
    }

    @Override
    public boolean update(SegmentHost host) {
        double elapsed = host.time() - startTime;

        double referenceHeading = MathUtil.normalizeAngle(startHeading + profile.position(elapsed));
        double referenceRate = profile.velocity(elapsed);

        double error = MathUtil.angleDelta(host.getPose().getHeading(), referenceHeading);
        double angular = referenceRate + headingPid.calculate(error, host.dt());
        host.drive(new ChassisSpeeds(0.0, angular));

        if (elapsed >= timeout) {
            return true;
        }
        if (elapsed < profile.getDuration()) {
            return false;
        }

        double finalError = Math.abs(MathUtil.angleDelta(host.getPose().getHeading(), resolvedTarget));
        boolean settled = finalError < headingTolerance
                && Math.abs(host.getAngularVelocity()) < velocityTolerance;

        if (!settled) {
            inToleranceSince = -1.0;
            return false;
        }
        if (inToleranceSince < 0.0) {
            inToleranceSince = host.time();
        }
        return host.time() - inToleranceSince >= settleTime;
    }

    @Override
    public Pose plannedEndPose(Pose plannedStartPose) {
        double heading = absolute
                ? MathUtil.normalizeAngle(targetHeading)
                : MathUtil.normalizeAngle(plannedStartPose.getHeading() + targetHeading);
        return plannedStartPose.withHeading(heading);
    }

    @Override
    public String describe() {
        return String.format("turn to %.0f deg", MathUtil.toDegrees(
                absolute ? targetHeading : resolvedTarget));
    }
}
