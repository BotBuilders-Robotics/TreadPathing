package org.treadpathing.holonomic;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.spline.SplinePath;

/**
 * Turns a path plus a heading plan into a holonomic trajectory.
 *
 * <p>Structurally the same two-pass sweep the tank generator runs, and deliberately so: walk
 * the path accelerating as hard as allowed, walk it back decelerating, take the lower of the
 * two. What changes is the ceiling at each station. There it comes from the curvature alone;
 * here it comes from the curvature <b>and</b> the angle between travel and the nose, because
 * that angle decides how much of the wheel budget translation costs and how much is left for
 * the turn the heading plan is asking for.
 *
 * <p>Everything the path itself knows -- arc length, tangent, curvature -- is reused from
 * {@code SplinePath} unchanged. The geometry was never the differential-drive part.
 */
public final class HolonomicTrajectoryGenerator {

    public static final double DEFAULT_SAMPLE_SPACING = 0.25;

    private static final int MAX_SAMPLES = 4000;

    private HolonomicTrajectoryGenerator() {
    }

    public static HolonomicTrajectory generate(SplinePath path, HeadingPlan headingPlan,
                                               HolonomicConstraints constraints,
                                               MecanumKinematics kinematics) {
        return generate(path, headingPlan, constraints, kinematics, DEFAULT_SAMPLE_SPACING);
    }

    public static HolonomicTrajectory generate(SplinePath path, HeadingPlan headingPlan,
                                               HolonomicConstraints constraints,
                                               MecanumKinematics kinematics,
                                               double sampleSpacing) {
        constraints.validate();

        double length = path.length();
        if (length < 1e-4) {
            throw new IllegalArgumentException(
                    "Path has effectively zero length; use a turn in place instead");
        }

        int intervals = MathUtil.clamp((int) Math.ceil(length / Math.max(sampleSpacing, 1e-3)),
                1, MAX_SAMPLES);
        int count = intervals + 1;

        double[] s = new double[count];
        double[] x = new double[count];
        double[] y = new double[count];
        double[] tangent = new double[count];
        double[] curvature = new double[count];
        double[] heading = new double[count];
        double[] headingRate = new double[count];
        double[] travelAngle = new double[count];
        double[] ceiling = new double[count];
        double[] v = new double[count];

        for (int i = 0; i < count; i++) {
            double arc = length * i / intervals;
            s[i] = arc;
            Pose p = path.poseAt(arc);
            x[i] = p.getX();
            y[i] = p.getY();
            tangent[i] = p.getHeading();
            curvature[i] = path.curvatureAt(arc);
            heading[i] = headingPlan.headingAt(path, arc);
            headingRate[i] = headingPlan.rateAt(path, arc);
            travelAngle[i] = MathUtil.normalizeAngle(tangent[i] - heading[i]);
            ceiling[i] = constraints.velocityLimit(
                    curvature[i], travelAngle[i], headingRate[i], kinematics);
        }

        // Pass 1: forward, accelerating.
        v[0] = Math.min(constraints.getStartVelocity(), ceiling[0]);
        for (int i = 1; i < count; i++) {
            double ds = s[i] - s[i - 1];
            double reachable = MathUtil.reachableVelocity(v[i - 1],
                    constraints.accelerationLimit(travelAngle[i - 1], headingRate[i - 1], kinematics),
                    ds);
            v[i] = Math.min(ceiling[i], reachable);
        }

        // Pass 2: backward, decelerating.
        v[count - 1] = Math.min(v[count - 1], constraints.getEndVelocity());
        for (int i = count - 2; i >= 0; i--) {
            double ds = s[i + 1] - s[i];
            double reachable = MathUtil.reachableVelocity(v[i + 1],
                    constraints.decelerationLimit(travelAngle[i], headingRate[i], kinematics),
                    ds);
            v[i] = Math.min(v[i], reachable);
        }

        // Pass 3: integrate time.
        double[] times = new double[count];
        double[] accels = new double[count];
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
                throw new IllegalStateException("Trajectory stalled at sample " + i
                        + "; the heading plan may be asking for more turn than the wheels have");
            }
            times[i] = times[i - 1] + dt;
            accels[i] = a;
        }
        accels[0] = count > 1 ? accels[1] : 0.0;

        // Pass 4: pack. Velocity is field-frame, because that is the frame the controller
        // corrects in and the frame a holonomic robot actually moves in.
        HolonomicSample[] samples = new HolonomicSample[count];
        for (int i = 0; i < count; i++) {
            samples[i] = new HolonomicSample(
                    times[i],
                    s[i],
                    new Pose(x[i], y[i], heading[i]),
                    v[i] * Math.cos(tangent[i]),
                    v[i] * Math.sin(tangent[i]),
                    v[i] * headingRate[i],
                    accels[i]);
        }

        return new HolonomicTrajectory(samples, length, headingPlan.describe());
    }
}
