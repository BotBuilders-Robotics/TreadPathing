package org.treadpathing.spline;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Vector2;

/**
 * Bidirectional lookup between a segment's spline parameter t and its arc length s.
 *
 * <p>There is no closed form for the arc length of a quintic, so the table is built once by
 * dense chord sampling and then interpolated. Everything downstream — the trajectory
 * parameterizer, the follower, the visualizer export — works in arc length, because t is not
 * proportional to distance and treating it as though it were produces a robot that
 * mysteriously speeds up and slows down on a constant-radius arc.
 */
public final class ArcLengthTable {

    /** Samples per segment. 400 puts the table under a millimetre of error on FTC-scale paths. */
    public static final int DEFAULT_SAMPLES = 400;

    private final double[] arcLengths;
    private final int samples;
    private final double totalLength;

    public ArcLengthTable(SplineSegment segment) {
        this(segment, DEFAULT_SAMPLES);
    }

    public ArcLengthTable(SplineSegment segment, int samples) {
        if (samples < 2) {
            throw new IllegalArgumentException("ArcLengthTable needs at least 2 samples");
        }
        this.samples = samples;
        this.arcLengths = new double[samples + 1];

        double accumulated = 0.0;
        Vector2 previous = segment.point(0.0);
        arcLengths[0] = 0.0;
        for (int i = 1; i <= samples; i++) {
            double t = (double) i / (double) samples;
            Vector2 current = segment.point(t);
            accumulated += current.distanceTo(previous);
            arcLengths[i] = accumulated;
            previous = current;
        }
        this.totalLength = accumulated;
    }

    public double length() {
        return totalLength;
    }

    /** Arc length travelled by parameter t. */
    public double arcLengthAt(double t) {
        double clamped = MathUtil.clamp(t, 0.0, 1.0);
        double scaled = clamped * samples;
        int index = (int) Math.floor(scaled);
        if (index >= samples) {
            return totalLength;
        }
        double fraction = scaled - index;
        return MathUtil.lerp(arcLengths[index], arcLengths[index + 1], fraction);
    }

    /** Inverse lookup: the parameter t at which the segment has travelled {@code s} inches. */
    public double parameterAt(double s) {
        if (totalLength < MathUtil.EPSILON) {
            return 0.0;
        }
        if (s <= 0.0) {
            return 0.0;
        }
        if (s >= totalLength) {
            return 1.0;
        }

        int low = 0;
        int high = samples;
        while (high - low > 1) {
            int mid = (low + high) / 2;
            if (arcLengths[mid] <= s) {
                low = mid;
            } else {
                high = mid;
            }
        }

        double span = arcLengths[high] - arcLengths[low];
        double fraction = span < MathUtil.EPSILON ? 0.0 : (s - arcLengths[low]) / span;
        return (low + fraction) / samples;
    }
}
