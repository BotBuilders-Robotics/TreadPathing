package org.treadpathing.trajectory;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/** A time-parameterised, feasibility-checked motion along one path. */
public final class Trajectory {

    private final TrajectorySample[] samples;
    private final boolean reversed;
    private final double duration;
    private final double length;

    public Trajectory(TrajectorySample[] samples, boolean reversed) {
        if (samples == null || samples.length < 2) {
            throw new IllegalArgumentException("A trajectory needs at least two samples");
        }
        this.samples = samples;
        this.reversed = reversed;
        this.duration = samples[samples.length - 1].getTime();
        this.length = samples[samples.length - 1].getArcLength();
    }

    public double getDuration() {
        return duration;
    }

    public double getLength() {
        return length;
    }

    public boolean isReversed() {
        return reversed;
    }

    public int sampleCount() {
        return samples.length;
    }

    public TrajectorySample rawSample(int index) {
        return samples[MathUtil.clamp(index, 0, samples.length - 1)];
    }

    public Pose startPose() {
        return samples[0].getPose();
    }

    public Pose endPose() {
        return samples[samples.length - 1].getPose();
    }

    /** The reference the follower should be tracking at elapsed time {@code t}. */
    public TrajectorySample sample(double t) {
        if (t <= 0.0) {
            return samples[0];
        }
        if (t >= duration) {
            return samples[samples.length - 1];
        }

        int low = 0;
        int high = samples.length - 1;
        while (high - low > 1) {
            int mid = (low + high) / 2;
            if (samples[mid].getTime() <= t) {
                low = mid;
            } else {
                high = mid;
            }
        }

        double span = samples[high].getTime() - samples[low].getTime();
        if (span < MathUtil.EPSILON) {
            return samples[low];
        }
        double fraction = (t - samples[low].getTime()) / span;
        return samples[low].interpolate(samples[high], fraction);
    }

    /** How far along the trajectory, as a fraction of arc length in [0, 1]. */
    public double completionAt(double t) {
        if (length < MathUtil.EPSILON) {
            return 1.0;
        }
        return MathUtil.clamp(sample(t).getArcLength() / length, 0.0, 1.0);
    }
}
