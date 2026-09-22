package org.treadpathing.route;

import org.treadpathing.control.ChassisSpeeds;
import org.treadpathing.geometry.Pose;

/**
 * Runs an action to completion while holding position, blocking the route until it finishes.
 *
 * <p>Holding rather than coasting matters: an idle drivetrain on a foam tile drifts, and a
 * two-second scoring action is long enough for that drift to move the robot out of range of
 * whatever it is scoring on.
 */
public final class ActionSegment extends Segment {

    private final Action action;
    private final double timeout;
    private final String label;

    private Pose anchor;
    private double startTime;
    private boolean started;
    private boolean finished;

    public ActionSegment(Action action, double timeout, String label) {
        if (action == null) {
            throw new IllegalArgumentException("Action must not be null");
        }
        this.action = action;
        this.timeout = timeout;
        this.label = label;
    }

    @Override
    public void init(SegmentHost host) {
        anchor = host.getPose();
        startTime = host.time();
        started = false;
        finished = false;
        host.getPoseHoldController().reset();
    }

    @Override
    public boolean update(SegmentHost host) {
        host.drive(hold(host));

        if (!started) {
            action.start();
            started = true;
        }
        if (!finished) {
            finished = action.run();
        }
        return finished || (host.time() - startTime) >= timeout;
    }

    private ChassisSpeeds hold(SegmentHost host) {
        return host.getPoseHoldController().calculate(host.getPose(), anchor, host.dt());
    }

    @Override
    public Pose plannedEndPose(Pose plannedStartPose) {
        return plannedStartPose;
    }

    @Override
    public String describe() {
        return label;
    }
}
