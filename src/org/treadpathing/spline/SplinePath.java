package org.treadpathing.spline;

import java.util.ArrayList;
import java.util.List;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.geometry.Vector2;

/**
 * A chain of quintic Hermite segments addressed by arc length.
 *
 * <p>The path knows nothing about time, velocity or the robot. It answers three questions at
 * any distance along itself: where, which way, and how sharply it is bending.
 */
public final class SplinePath {

    /**
     * Tangent handle length as a multiple of the straight-line distance between waypoints.
     * Larger values bulge the curve outward and make it approach each waypoint more squarely;
     * smaller values tighten it toward the chord.
     */
    public static final double DEFAULT_TANGENT_SCALE = 1.25;

    /** A leg with this cap is not capped at all; the route constraints alone govern it. */
    public static final double NO_SPEED_CAP = 0.0;

    private final SplineSegment[] segments;
    private final ArcLengthTable[] tables;
    private final double[] segmentStart;
    private final double[] segmentSpeedCap;
    private final double totalLength;

    private SplinePath(SplineSegment[] segments, ArcLengthTable[] tables, double[] speedCaps) {
        this.segments = segments;
        this.tables = tables;
        this.segmentSpeedCap = speedCaps;
        this.segmentStart = new double[segments.length];

        double accumulated = 0.0;
        for (int i = 0; i < segments.length; i++) {
            segmentStart[i] = accumulated;
            accumulated += tables[i].length();
        }
        this.totalLength = accumulated;
    }

    public static Builder builder(Pose start) {
        return new Builder(start);
    }

    /** Convenience: a straight line between two points, travelling along the chord. */
    public static SplinePath line(Pose start, Pose end) {
        double heading = end.position().minus(start.position()).angle();
        return builder(new Pose(start.getX(), start.getY(), heading))
                .to(new Pose(end.getX(), end.getY(), heading))
                .build();
    }

    public double length() {
        return totalLength;
    }

    public int segmentCount() {
        return segments.length;
    }

    public SplineSegment segment(int index) {
        return segments[index];
    }

    public Pose startPose() {
        return segments[0].getStart();
    }

    public Pose endPose() {
        return segments[segments.length - 1].getEnd();
    }

    public Vector2 pointAt(double arcLength) {
        int index = segmentIndexAt(arcLength);
        double local = tables[index].parameterAt(arcLength - segmentStart[index]);
        return segments[index].point(local);
    }

    /** Direction of travel at this arc length. */
    public double tangentAngleAt(double arcLength) {
        int index = segmentIndexAt(arcLength);
        double local = tables[index].parameterAt(arcLength - segmentStart[index]);
        return segments[index].tangentAngle(local);
    }

    /** Signed curvature at this arc length, in inverse inches. */
    public double curvatureAt(double arcLength) {
        int index = segmentIndexAt(arcLength);
        double local = tables[index].parameterAt(arcLength - segmentStart[index]);
        return segments[index].curvature(local);
    }

    /**
     * The speed ceiling asked for on the leg containing this arc length, or
     * {@link #NO_SPEED_CAP} if that leg is uncapped.
     *
     * <p>This is a property of the path rather than of the constraints because it varies
     * along the path: it is how you ask for one slow leg in the middle of a fast route. The
     * generator folds it into the same forward/backward sweep as every other limit, so the
     * robot eases down into a capped leg and back up out of it instead of stopping at the
     * seam.
     */
    public double speedCapAt(double arcLength) {
        return segmentSpeedCap[segmentIndexAt(arcLength)];
    }

    /** Position plus direction of travel at this arc length. */
    public Pose poseAt(double arcLength) {
        int index = segmentIndexAt(arcLength);
        double local = tables[index].parameterAt(arcLength - segmentStart[index]);
        return segments[index].poseAt(local);
    }

    private int segmentIndexAt(double arcLength) {
        double s = MathUtil.clamp(arcLength, 0.0, totalLength);
        for (int i = segments.length - 1; i >= 0; i--) {
            if (s >= segmentStart[i]) {
                return i;
            }
        }
        return 0;
    }

    /** Incremental builder. Each {@code to} call appends one quintic segment. */
    public static final class Builder {

        private final List<Pose> waypoints = new ArrayList<Pose>();
        private final List<Double> outgoingScale = new ArrayList<Double>();
        private final List<Double> incomingScale = new ArrayList<Double>();
        private final List<Double> speedCap = new ArrayList<Double>();

        private Builder(Pose start) {
            waypoints.add(start);
            outgoingScale.add(Double.valueOf(DEFAULT_TANGENT_SCALE));
            incomingScale.add(Double.valueOf(DEFAULT_TANGENT_SCALE));
            speedCap.add(Double.valueOf(NO_SPEED_CAP));
        }

        public Builder to(Pose waypoint) {
            return to(waypoint, DEFAULT_TANGENT_SCALE, DEFAULT_TANGENT_SCALE);
        }

        public Builder to(Pose waypoint, double tangentScale) {
            return to(waypoint, tangentScale, tangentScale);
        }

        /**
         * @param incoming scale on the tangent handle arriving at this waypoint
         * @param outgoing scale on the tangent handle leaving it
         */
        public Builder to(Pose waypoint, double incoming, double outgoing) {
            return to(waypoint, incoming, outgoing, NO_SPEED_CAP);
        }

        /**
         * @param maxSpeed speed ceiling for the leg arriving at this waypoint, in inches per
         *                 second, or {@link #NO_SPEED_CAP} for none
         */
        public Builder to(Pose waypoint, double incoming, double outgoing, double maxSpeed) {
            waypoints.add(waypoint);
            incomingScale.add(Double.valueOf(incoming));
            outgoingScale.add(Double.valueOf(outgoing));
            speedCap.add(Double.valueOf(maxSpeed));
            return this;
        }

        public int waypointCount() {
            return waypoints.size();
        }

        public SplinePath build() {
            if (waypoints.size() < 2) {
                throw new IllegalStateException("A path needs at least two waypoints");
            }
            int count = waypoints.size() - 1;
            SplineSegment[] segments = new SplineSegment[count];
            ArcLengthTable[] tables = new ArcLengthTable[count];
            double[] caps = new double[count];

            for (int i = 0; i < count; i++) {
                Pose a = waypoints.get(i);
                Pose b = waypoints.get(i + 1);
                double chord = a.distanceTo(b);
                if (chord < 1e-6) {
                    throw new IllegalStateException(
                            "Waypoints " + i + " and " + (i + 1) + " are on top of each other: " + a);
                }
                double outMag = chord * outgoingScale.get(i).doubleValue();
                double inMag = chord * incomingScale.get(i + 1).doubleValue();
                segments[i] = new SplineSegment(a, outMag, b, inMag);
                tables[i] = new ArcLengthTable(segments[i]);
                // The cap rides on the waypoint you are driving to, so leg i takes it from
                // waypoint i + 1.
                caps[i] = Math.max(speedCap.get(i + 1).doubleValue(), 0.0);
            }
            return new SplinePath(segments, tables, caps);
        }
    }
}
