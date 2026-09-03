package org.treadpathing.trajectory;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.spline.SplinePath;

/**
 * Turns a {@link SplinePath} into a {@link Trajectory} with a two-pass velocity profile.
 *
 * <p>The algorithm is the standard forward/backward sweep: walk the path once accelerating
 * as hard as the constraints allow, walk it back once decelerating, and take the lower of
 * the two at every point. What is left is a speed profile that is feasible everywhere and as
 * fast as the envelope permits.
 *
 * <p>Generation costs a few milliseconds and allocates. It runs during {@code init()}, never
 * inside the control loop.
 */
public final class TrajectoryGenerator {

    /** Distance between profile samples, in inches. */
    public static final double DEFAULT_SAMPLE_SPACING = 0.25;

    private static final int MAX_SAMPLES = 4000;

    private TrajectoryGenerator() {
    }

    public static Trajectory generate(SplinePath path, TrajectoryConstraints constraints, boolean reversed) {
        return generate(path, constraints, reversed, DEFAULT_SAMPLE_SPACING);
    }

    public static Trajectory generate(SplinePath path, TrajectoryConstraints constraints,
                                      boolean reversed, double sampleSpacing) {
        constraints.validate();

        double length = path.length();
        if (length < 1e-4) {
            throw new IllegalArgumentException("Path has effectively zero length; use a turn or hold instead");
        }

        int intervals = (int) Math.ceil(length / Math.max(sampleSpacing, 1e-3));
        intervals = MathUtil.clamp(intervals, 1, MAX_SAMPLES);
        int count = intervals + 1;

        double[] s = new double[count];
        double[] curvature = new double[count];
        double[] tangent = new double[count];
        double[] x = new double[count];
        double[] y = new double[count];
        double[] v = new double[count];

        for (int i = 0; i < count; i++) {
            double arc = length * i / intervals;
            s[i] = arc;
            Pose p = path.poseAt(arc);
            x[i] = p.getX();
            y[i] = p.getY();
            tangent[i] = p.getHeading();
            curvature[i] = path.curvatureAt(arc);
        }

        // Pass 1: forward, accelerating.
        v[0] = Math.min(constraints.getStartVelocity(), constraints.velocityLimit(curvature[0]));
        for (int i = 1; i < count; i++) {
            double ds = s[i] - s[i - 1];
            double reachable = MathUtil.reachableVelocity(
                    v[i - 1], constraints.accelerationLimit(curvature[i - 1]), ds);
            v[i] = Math.min(constraints.velocityLimit(curvature[i]), reachable);
        }

        // Pass 2: backward, decelerating.
        v[count - 1] = Math.min(v[count - 1], constraints.getEndVelocity());
        for (int i = count - 2; i >= 0; i--) {
            double ds = s[i + 1] - s[i];
            double reachable = MathUtil.reachableVelocity(
                    v[i + 1], constraints.decelerationLimit(curvature[i]), ds);
            v[i] = Math.min(v[i], reachable);
        }

        // Pass 3: integrate time and back out acceleration.
        double[] times = new double[count];
        double[] accels = new double[count];
        times[0] = 0.0;
        for (int i = 1; i < count; i++) {
            double ds = s[i] - s[i - 1];
            double a = (v[i] * v[i] - v[i - 1] * v[i - 1]) / (2.0 * ds);
            double dt;
            if (Math.abs(a) > 1e-6) {
                dt = (v[i] - v[i - 1]) / a;
            } else if (v[i - 1] > 1e-6) {
                dt = ds / v[i - 1];
            } else if (v[i] > 1e-6) {
                dt = ds / v[i];
            } else {
                // Both endpoints are stopped over a real distance. Only reachable if the
                // constraints forbid moving at all, which validate() already rules out.
                throw new IllegalStateException(
                        "Trajectory stalled at sample " + i + "; check maxAcceleration and the "
                                + "start/end velocities");
            }
            times[i] = times[i - 1] + dt;
            accels[i] = a;
        }
        accels[0] = accels.length > 1 ? accels[1] : 0.0;

        // Pass 4: apply direction and pack.
        double directionSign = reversed ? -1.0 : 1.0;
        double headingOffset = reversed ? Math.PI : 0.0;

        TrajectorySample[] samples = new TrajectorySample[count];
        for (int i = 0; i < count; i++) {
            Pose robotPose = new Pose(x[i], y[i], MathUtil.normalizeAngle(tangent[i] + headingOffset));
            samples[i] = new TrajectorySample(
                    times[i],
                    s[i],
                    robotPose,
                    directionSign * v[i],
                    directionSign * accels[i],
                    directionSign * curvature[i]);
        }
        return new Trajectory(samples, reversed);
    }
}
