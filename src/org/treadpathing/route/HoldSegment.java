package org.treadpathing.route;

import org.treadpathing.control.ChassisSpeeds;
import org.treadpathing.geometry.Pose;

/**
 * Converges on an exact pose and holds it.
 *
 * <p>This is the segment that sets a route's terminal accuracy. A trajectory follower cannot
 * do it: its reference velocity is zero at the end, and a differential drive has no lateral
 * authority at zero velocity. Put one of these wherever the auto needs to be somewhere
 * precisely — before a scoring action, at the end of the route — and treat the drive
 * segments as getting the robot to roughly the right place quickly.
 *
 * <p>Exits early once it is inside tolerance and has stayed there for the settle time, so a
 * generous timeout costs nothing when the robot arrives cleanly.
 */
public final class HoldSegment extends Segment {

    private final Pose target;
    private final double timeout;
    private final double settleTime;
    private final boolean holdForFullDuration;

    private double startTime;
    private double inToleranceSince;

    /**
     * @param target              pose to converge on; null means "hold wherever you are now"
     * @param timeout             seconds to spend before giving up and moving on
     * @param settleTime          seconds inside tolerance required before declaring success
     * @param holdForFullDuration true to stay for the whole timeout even once converged,
     *                            which is what you want while a mechanism is scoring
     */
    public HoldSegment(Pose target, double timeout, double settleTime, boolean holdForFullDuration) {
        this.target = target;
        this.timeout = timeout;
        this.settleTime = settleTime;
        this.holdForFullDuration = holdForFullDuration;
    }

    private Pose resolved;

    @Override
    public void init(SegmentHost host) {
        resolved = target != null ? target : host.getPose();
        startTime = host.time();
        inToleranceSince = -1.0;
        host.getPoseHoldController().reset();
    }

    @Override
    public boolean update(SegmentHost host) {
        double elapsed = host.time() - startTime;

        ChassisSpeeds command = host.getPoseHoldController()
                .calculate(host.getPose(), resolved, host.dt());
        host.drive(command);

        if (elapsed >= timeout) {
            return true;
        }
        if (holdForFullDuration) {
            return false;
        }

        if (!host.getPoseHoldController().atTarget(host.getPose(), resolved)) {
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
        return target != null ? target : plannedStartPose;
    }

    @Override
    public String describe() {
        return target != null ? "hold " + target : "hold pose";
    }
}
