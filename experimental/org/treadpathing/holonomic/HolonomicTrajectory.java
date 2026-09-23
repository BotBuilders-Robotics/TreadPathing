package org.treadpathing.holonomic;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/** A time-parameterised holonomic trajectory: the same shape as the tank one, wider samples. */
public final class HolonomicTrajectory {

    private final HolonomicSample[] samples;
    private final double duration;
    private final double length;
    private final String headingPlan;

    HolonomicTrajectory(HolonomicSample[] samples, double length, String headingPlan) {
        this.samples = samples;
        this.duration = samples[samples.length - 1].getTime();
        this.length = length;
        this.headingPlan = headingPlan;
    }

    public double getDuration() {
        return duration;
    }

    public double getLength() {
        return length;
    }

    public String getHeadingPlan() {
        return headingPlan;
    }

    public int sampleCount() {
        return samples.length;
    }

    public HolonomicSample rawSample(int index) {
        return samples[MathUtil.clamp(index, 0, samples.length - 1)];
    }

    public Pose endPose() {
        return samples[samples.length - 1].getPose();
    }

    /** The reference at a time, linearly interpolated and clamped at both ends. */
    public HolonomicSample sample(double time) {
        if (time <= 0.0) {
            return samples[0];
        }
        if (time >= duration) {
            return samples[samples.length - 1];
        }

        int low = 0;
        int high = samples.length - 1;
        while (high - low > 1) {
            int mid = (low + high) >>> 1;
            if (samples[mid].getTime() <= time) {
                low = mid;
            } else {
                high = mid;
            }
        }

        HolonomicSample a = samples[low];
        HolonomicSample b = samples[high];
        double span = b.getTime() - a.getTime();
        double t = span > 1e-9 ? (time - a.getTime()) / span : 0.0;

        Pose pose = new Pose(
                MathUtil.lerp(a.getPose().getX(), b.getPose().getX(), t),
                MathUtil.lerp(a.getPose().getY(), b.getPose().getY(), t),
                MathUtil.lerpAngle(a.getPose().getHeading(), b.getPose().getHeading(), t));

        return new HolonomicSample(time,
                MathUtil.lerp(a.getArcLength(), b.getArcLength(), t),
                pose,
                MathUtil.lerp(a.getFieldVx(), b.getFieldVx(), t),
                MathUtil.lerp(a.getFieldVy(), b.getFieldVy(), t),
                MathUtil.lerp(a.getOmega(), b.getOmega(), t),
                MathUtil.lerp(a.getAcceleration(), b.getAcceleration(), t),
                MathUtil.lerp(a.getFieldAx(), b.getFieldAx(), t),
                MathUtil.lerp(a.getFieldAy(), b.getFieldAy(), t),
                MathUtil.lerp(a.getAlpha(), b.getAlpha(), t));
    }

    public double completionAt(double time) {
        if (length < 1e-9) {
            return 1.0;
        }
        return MathUtil.clamp(sample(time).getArcLength() / length, 0.0, 1.0);
    }
}
