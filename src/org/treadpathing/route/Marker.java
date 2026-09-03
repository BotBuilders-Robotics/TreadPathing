package org.treadpathing.route;

/**
 * An action fired part-way through a drive segment, without pausing the drive.
 *
 * <p>The trigger is a fraction of the segment's <b>arc length</b>, not of its duration, so a
 * marker at 0.6 fires six tenths of the way along the path regardless of where the velocity
 * profile happened to slow down.
 *
 * <p>Once fired, the action keeps being polled by the follower even after the segment ends,
 * so an arm that takes longer to raise than the drive takes to finish does not get cut off.
 */
public final class Marker {

    private final double completion;
    private final Action action;

    public Marker(double completion, Action action) {
        if (action == null) {
            throw new IllegalArgumentException("Marker action must not be null");
        }
        if (completion < 0.0 || completion > 1.0) {
            throw new IllegalArgumentException("Marker completion must be between 0 and 1");
        }
        this.completion = completion;
        this.action = action;
    }

    public double getCompletion() {
        return completion;
    }

    public Action getAction() {
        return action;
    }
}
