package org.treadpathing.route;

import org.treadpathing.geometry.Pose;

/** Stops the drivetrain and waits. Use {@code stopAndHold} instead if position matters. */
public final class WaitSegment extends Segment {

    private final double duration;
    private double startTime;

    public WaitSegment(double seconds) {
        this.duration = seconds;
    }

    @Override
    public void init(SegmentHost host) {
        startTime = host.time();
    }

    @Override
    public boolean update(SegmentHost host) {
        host.stopDrive();
        return host.time() - startTime >= duration;
    }

    @Override
    public Pose plannedEndPose(Pose plannedStartPose) {
        return plannedStartPose;
    }

    @Override
    public String describe() {
        return String.format("wait %.2fs", duration);
    }
}
