package org.treadpathing.holonomic;

import java.util.Collections;
import java.util.List;

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
 *
 * <p>Several pieces, each a path with its own heading plan, can be solved as one trajectory.
 * That is how a route changes what the nose is doing without stopping: the sweep runs across
 * the seam as if it were any other station, so the robot carries its speed straight through.
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
        return generate(Collections.singletonList(path), Collections.singletonList(headingPlan),
                constraints, kinematics, sampleSpacing);
    }

    /**
     * One trajectory across several pieces driven back to back without stopping.
     *
     * <p>Each piece's path should start where the last one ended and leave in the direction it
     * arrived, and each plan should start at the heading the last one finished on. The route
     * builder guarantees both; nothing here can repair a corner or a heading step, it can only
     * profile what it is given.
     */
    public static HolonomicTrajectory generate(List<SplinePath> paths, List<HeadingPlan> plans,
                                               HolonomicConstraints constraints,
                                               MecanumKinematics kinematics,
                                               double sampleSpacing) {
        constraints.validate();
        if (paths.isEmpty() || paths.size() != plans.size()) {
            throw new IllegalArgumentException("Need one heading plan for every path");
        }

        int pieces = paths.size();
        int[] intervals = new int[pieces];
        int count = 1;
        double totalLength = 0.0;
        for (int p = 0; p < pieces; p++) {
            double length = paths.get(p).length();
            if (length < 1e-4) {
                throw new IllegalArgumentException(
                        "Path has effectively zero length; use a turn in place instead");
            }
            intervals[p] = MathUtil.clamp(
                    (int) Math.ceil(length / Math.max(sampleSpacing, 1e-3)), 1, MAX_SAMPLES);
            count += intervals[p];
            totalLength += length;
        }

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

        // Stations, piece by piece. A seam is one station, not two: the first station of each
        // later piece is skipped, and the shared one takes the stricter ceiling of the two
        // sides, since the robot has to satisfy both plans at the instant it crosses.
        int i = 0;
        double offset = 0.0;
        for (int p = 0; p < pieces; p++) {
            SplinePath path = paths.get(p);
            HeadingPlan plan = plans.get(p);
            double length = path.length();
            for (int j = p == 0 ? 0 : 1; j <= intervals[p]; j++) {
                double arc = length * j / intervals[p];
                s[i] = offset + arc;
                Pose pose = path.poseAt(arc);
                x[i] = pose.getX();
                y[i] = pose.getY();
                tangent[i] = pose.getHeading();
                curvature[i] = path.curvatureAt(arc);
                heading[i] = plan.headingAt(path, arc);
                headingRate[i] = plan.rateAt(path, arc);
                travelAngle[i] = MathUtil.normalizeAngle(tangent[i] - heading[i]);
                ceiling[i] = constraints.velocityLimit(
                        curvature[i], travelAngle[i], headingRate[i], kinematics);
                i++;
            }
            if (p + 1 < pieces) {
                SplinePath next = paths.get(p + 1);
                HeadingPlan nextPlan = plans.get(p + 1);
                double nextRate = nextPlan.rateAt(next, 0.0);
                double nextTravel = MathUtil.normalizeAngle(
                        next.poseAt(0.0).getHeading() - nextPlan.headingAt(next, 0.0));
                double nextCeiling = constraints.velocityLimit(
                        next.curvatureAt(0.0), nextTravel, nextRate, kinematics);
                ceiling[i - 1] = Math.min(ceiling[i - 1], nextCeiling);
            }
            offset += length;
        }

        // Pass 1: forward, accelerating.
        v[0] = Math.min(constraints.getStartVelocity(), ceiling[0]);
        for (i = 1; i < count; i++) {
            double ds = s[i] - s[i - 1];
            double reachable = MathUtil.reachableVelocity(v[i - 1],
                    constraints.accelerationLimit(travelAngle[i - 1], headingRate[i - 1], kinematics),
                    ds);
            v[i] = Math.min(ceiling[i], reachable);
        }

        // Pass 2: backward, decelerating.
        v[count - 1] = Math.min(v[count - 1], constraints.getEndVelocity());
        for (i = count - 2; i >= 0; i--) {
            double ds = s[i + 1] - s[i];
            double reachable = MathUtil.reachableVelocity(v[i + 1],
                    constraints.decelerationLimit(travelAngle[i], headingRate[i], kinematics),
                    ds);
            v[i] = Math.min(v[i], reachable);
        }

        // Pass 3: integrate time.
        double[] times = new double[count];
        double[] accels = new double[count];
        for (i = 1; i < count; i++) {
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
        // corrects in and the frame a holonomic robot actually moves in. Acceleration goes
        // with it as a vector -- the part along the path plus the part into the bend -- so the
        // drive can work out which way each wheel is being asked to speed up.
        HolonomicSample[] samples = new HolonomicSample[count];
        for (i = 0; i < count; i++) {
            double cos = Math.cos(tangent[i]);
            double sin = Math.sin(tangent[i]);
            double centripetal = v[i] * v[i] * curvature[i];
            samples[i] = new HolonomicSample(
                    times[i],
                    s[i],
                    new Pose(x[i], y[i], heading[i]),
                    v[i] * cos,
                    v[i] * sin,
                    v[i] * headingRate[i],
                    accels[i],
                    accels[i] * cos - centripetal * sin,
                    accels[i] * sin + centripetal * cos,
                    accels[i] * headingRate[i]);
        }

        return new HolonomicTrajectory(samples, totalLength, describe(plans));
    }

    private static String describe(List<HeadingPlan> plans) {
        StringBuilder out = new StringBuilder();
        for (int p = 0; p < plans.size(); p++) {
            if (p > 0) {
                out.append(", then ");
            }
            out.append(plans.get(p).describe());
        }
        return out.toString();
    }
}
