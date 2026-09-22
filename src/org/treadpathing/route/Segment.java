package org.treadpathing.route;

import org.treadpathing.geometry.Pose;

/**
 * One motion primitive in a route.
 *
 * <p>A tank route is a sequence of these rather than one continuous vector field, because
 * turning in place, reversing direction and converging on a final pose are physically
 * different manoeuvres. Pretending otherwise is where homegrown differential-drive followers
 * come apart.
 */
public abstract class Segment {

    /** Called once when the segment becomes active. */
    public abstract void init(SegmentHost host);

    /**
     * Called every loop while the segment is active.
     *
     * @return true when the segment is finished
     */
    public abstract boolean update(SegmentHost host);

    /**
     * Where the robot is planned to end up, given where it was planned to start. Used by
     * {@link RouteBuilder} to chain segments at build time; it is not consulted at run time.
     */
    public abstract Pose plannedEndPose(Pose plannedStartPose);

    /** Short human-readable label, shown in telemetry. */
    public abstract String describe();
}
