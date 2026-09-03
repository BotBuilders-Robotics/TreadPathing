package org.treadpathing.route;

import java.util.List;

import org.treadpathing.geometry.Pose;

/** A built, immutable sequence of motion primitives. */
public final class Route {

    private final Segment[] segments;
    private final Pose plannedStart;
    private final Pose plannedEnd;

    Route(List<Segment> segments, Pose plannedStart, Pose plannedEnd) {
        this.segments = segments.toArray(new Segment[segments.size()]);
        this.plannedStart = plannedStart;
        this.plannedEnd = plannedEnd;
    }

    public int size() {
        return segments.length;
    }

    public Segment get(int index) {
        return segments[index];
    }

    public Pose getPlannedStart() {
        return plannedStart;
    }

    public Pose getPlannedEnd() {
        return plannedEnd;
    }

    /** Multi-line summary, handy to dump to telemetry during init. */
    public String summary() {
        StringBuilder builder = new StringBuilder();
        builder.append("Route: ").append(segments.length).append(" segments\n");
        for (int i = 0; i < segments.length; i++) {
            builder.append("  ").append(i).append(". ").append(segments[i].describe()).append('\n');
        }
        builder.append("  ends at ").append(plannedEnd);
        return builder.toString();
    }
}
