package org.treadpathing.holonomic;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.spline.SplinePath;

/** The three heading plans worth having before anyone asks for a fourth. */
public final class HeadingPlans {

    private HeadingPlans() {
    }

    /** Nose fixed at one angle for the whole path: the strafe, and what a scorer wants. */
    public static HeadingPlan hold(final double headingRadians) {
        return new Hold(headingRadians);
    }

    /**
     * Nose along the direction of travel, which is what a tank drive does because it has no
     * choice. Worth having as a plan because it is often the right one anyway: it is the
     * cheapest in wheel speed, since no wheel budget goes on rotation beyond the turn the
     * corner itself demands.
     */
    public static HeadingPlan tangent() {
        return new Tangent(0.0);
    }

    /** Nose held at a fixed offset from the direction of travel. Offset pi is "reversed". */
    public static HeadingPlan tangent(double offsetRadians) {
        return new Tangent(offsetRadians);
    }

    /**
     * Turns steadily from one angle to another across the path, so the turn is spread over
     * the whole drive rather than bunched at one end.
     */
    public static HeadingPlan interpolate(double fromRadians, double toRadians) {
        return new Interpolate(fromRadians, toRadians);
    }

    /**
     * A nose angle at every waypoint, interpolated in between.
     *
     * <p>The plan the path editor writes, and the one that matches how a team describes an
     * auto out loud: "face the wall going out, be square to the basket by the time you get
     * there". Between waypoints the nose turns steadily; at a waypoint it is exactly what was
     * asked for.
     *
     * @param headings one per waypoint, so one more than the path has segments
     */
    public static HeadingPlan byWaypoints(double[] headings) {
        return new ByWaypoints(headings);
    }

    private static final class ByWaypoints implements HeadingPlan {
        private final double[] headings;

        ByWaypoints(double[] headings) {
            if (headings == null || headings.length < 2) {
                throw new IllegalArgumentException(
                        "A waypoint heading plan needs an angle for every waypoint");
            }
            this.headings = headings;
        }

        /** @return {index of the leg containing this distance, fraction along that leg} */
        private double[] locate(SplinePath path, double arcLength) {
            int last = Math.min(headings.length - 2, path.segmentCount() - 1);
            for (int i = last; i >= 0; i--) {
                double start = path.segmentStart(i);
                if (arcLength >= start || i == 0) {
                    double end = i < last ? path.segmentStart(i + 1) : path.length();
                    double span = end - start;
                    double fraction = span > 1e-9
                            ? MathUtil.clamp((arcLength - start) / span, 0.0, 1.0) : 0.0;
                    return new double[] {i, fraction, span};
                }
            }
            return new double[] {0, 0.0, path.length()};
        }

        @Override
        public double headingAt(SplinePath path, double arcLength) {
            double[] at = locate(path, arcLength);
            int leg = (int) at[0];
            return MathUtil.lerpAngle(headings[leg], headings[leg + 1], at[1]);
        }

        @Override
        public double rateAt(SplinePath path, double arcLength) {
            double[] at = locate(path, arcLength);
            int leg = (int) at[0];
            double span = at[2];
            return span > 1e-9
                    ? MathUtil.angleDelta(headings[leg], headings[leg + 1]) / span : 0.0;
        }

        @Override
        public String describe() {
            return "nose per waypoint";
        }
    }

    private static final class Hold implements HeadingPlan {
        private final double heading;

        Hold(double heading) {
            this.heading = heading;
        }

        @Override
        public double headingAt(SplinePath path, double arcLength) {
            return heading;
        }

        @Override
        public double rateAt(SplinePath path, double arcLength) {
            return 0.0;
        }

        @Override
        public String describe() {
            return String.format("hold %.0f deg", Math.toDegrees(heading));
        }
    }

    private static final class Tangent implements HeadingPlan {
        private final double offset;

        Tangent(double offset) {
            this.offset = offset;
        }

        @Override
        public double headingAt(SplinePath path, double arcLength) {
            return MathUtil.normalizeAngle(path.tangentAngleAt(arcLength) + offset);
        }

        @Override
        public double rateAt(SplinePath path, double arcLength) {
            // The tangent turns at exactly the path's curvature, by definition. Which is why
            // the tank library never needed this method: dtheta/ds and curvature are the
            // same number there.
            return path.curvatureAt(arcLength);
        }

        @Override
        public String describe() {
            return Math.abs(offset) < 1e-9 ? "tangent"
                    : String.format("tangent %+.0f deg", Math.toDegrees(offset));
        }
    }

    private static final class Interpolate implements HeadingPlan {
        private final double from;
        private final double delta;

        Interpolate(double from, double to) {
            this.from = from;
            this.delta = MathUtil.angleDelta(from, to);
        }

        @Override
        public double headingAt(SplinePath path, double arcLength) {
            double length = path.length();
            double fraction = length > 1e-9 ? MathUtil.clamp(arcLength / length, 0.0, 1.0) : 0.0;
            return MathUtil.normalizeAngle(from + delta * fraction);
        }

        @Override
        public double rateAt(SplinePath path, double arcLength) {
            double length = path.length();
            return length > 1e-9 ? delta / length : 0.0;
        }

        @Override
        public String describe() {
            return String.format("turn %+.0f deg across the path", Math.toDegrees(delta));
        }
    }
}
