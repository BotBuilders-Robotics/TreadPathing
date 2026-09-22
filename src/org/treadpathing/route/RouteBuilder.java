package org.treadpathing.route;

import java.util.ArrayList;
import java.util.List;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.spline.SplinePath;
import org.treadpathing.trajectory.TrajectoryConstraints;

/**
 * Builds a route out of motion primitives, reading like a description of the auto.
 *
 * <pre>
 * Route route = follower.route()
 *         .splineTo(new Pose(34.0, 60.0, 0.0))
 *         .splineTo(new Pose(52.0, 84.0, Math.toRadians(70)))
 *         .marker(0.60, raiseArm)
 *         .stopAndHold(1.5)
 *         .action(scoreSample)
 *         .turnTo(Math.toRadians(180))
 *         .reversed()
 *         .splineTo(new Pose(20.0, 72.0, Math.toRadians(180)))
 *         .build();
 * </pre>
 *
 * <h3>Headings are robot headings</h3>
 *
 * Every pose you pass is where the <b>robot's nose</b> points, which is what you see when you
 * look at the field. Behind the scenes a reversed segment travels along the opposite tangent;
 * the builder does that flip so you never have to write a heading pointing away from where
 * the robot is going.
 *
 * <h3>Cusps are automatic</h3>
 *
 * Calling {@link #reversed()} or {@link #forward()} closes off whatever path is being
 * accumulated and starts a new one. Because every trajectory begins and ends at rest, the
 * seam is dynamically feasible: the robot comes to a stop, then sets off the other way. That
 * is the only honest way for a differential drive to change direction, and doing it
 * explicitly beats discovering it as a mystery lurch mid-auto.
 */
public final class RouteBuilder {

    private final RouteDefaults defaults;
    private final Pose start;
    private final List<Segment> segments = new ArrayList<Segment>();

    private Pose cursor;
    private boolean reversed;
    private TrajectoryConstraints constraints;

    private SplinePath.Builder pendingPath;
    private List<Marker> pendingMarkers;
    private int pendingWaypoints;
    private boolean pendingReversed;

    public RouteBuilder(Pose start, RouteDefaults defaults) {
        this.start = start;
        this.cursor = start;
        this.defaults = defaults;
        this.constraints = defaults.getConstraints();
        this.reversed = false;
    }

    // ----- direction -------------------------------------------------------------------

    /** Subsequent waypoints are driven backwards. Closes the current path with a cusp. */
    public RouteBuilder reversed() {
        return setReversed(true);
    }

    /** Subsequent waypoints are driven forwards. Closes the current path with a cusp. */
    public RouteBuilder forward() {
        return setReversed(false);
    }

    public RouteBuilder setReversed(boolean value) {
        if (value != reversed) {
            flush();
            reversed = value;
        }
        return this;
    }

    // ----- limits ----------------------------------------------------------------------

    /** Applies to every drive segment after this call. Closes the current path. */
    public RouteBuilder constraints(TrajectoryConstraints newConstraints) {
        flush();
        this.constraints = newConstraints;
        return this;
    }

    // ----- driving ---------------------------------------------------------------------

    /** Adds a spline waypoint. The pose's heading is the robot's heading there. */
    public RouteBuilder splineTo(Pose waypoint) {
        return splineTo(waypoint, SplinePath.DEFAULT_TANGENT_SCALE);
    }

    /**
     * @param tangentScale handle length as a multiple of the distance from the previous
     *                     waypoint. Larger bulges the curve out; smaller hugs the chord.
     */
    public RouteBuilder splineTo(Pose waypoint, double tangentScale) {
        return splineTo(waypoint, tangentScale, SplinePath.NO_SPEED_CAP);
    }

    /**
     * Adds a spline waypoint that is approached no faster than {@code maxSpeed}.
     *
     * <p>The cap governs the leg <b>into</b> this waypoint and nothing after it, so one slow
     * approach in the middle of a fast route costs you only that leg. It is a ceiling, not a
     * target: curvature and the wheel-speed limit can still hold the robot below it. Unlike
     * {@link #constraints}, it does not close the path, so the robot eases down into the slow
     * leg and back up out of it rather than stopping at either end.
     *
     * @param maxSpeed inches per second, or {@link SplinePath#NO_SPEED_CAP} for no cap
     */
    public RouteBuilder splineTo(Pose waypoint, double tangentScale, double maxSpeed) {
        ensurePending();
        pendingPath.to(travelPose(waypoint), tangentScale, tangentScale, maxSpeed);
        pendingWaypoints++;
        cursor = waypoint;
        return this;
    }

    /**
     * Drives straight to a point. Because a differential drive cannot slide sideways onto a
     * line, a turn is inserted first if the robot is not already pointing along it.
     */
    public RouteBuilder lineTo(double x, double y) {
        return lineTo(x, y, SplinePath.NO_SPEED_CAP);
    }

    /** As {@link #lineTo(double, double)}, driven no faster than {@code maxSpeed}. */
    public RouteBuilder lineTo(double x, double y, double maxSpeed) {
        double chord = Math.atan2(y - cursor.getY(), x - cursor.getX());
        double currentTravel = travelHeading(cursor.getHeading());

        if (Math.abs(MathUtil.angleDelta(currentTravel, chord)) > defaults.getLineHeadingTolerance()) {
            turnTo(robotHeading(chord));
        }
        return splineTo(new Pose(x, y, robotHeading(chord)),
                SplinePath.DEFAULT_TANGENT_SCALE, maxSpeed);
    }

    /**
     * Fires an action part-way through the drive segment currently being built, without
     * pausing it.
     *
     * @param completion fraction of the segment's arc length, 0 to 1
     */
    public RouteBuilder marker(double completion, Action action) {
        if (pendingPath == null) {
            throw new IllegalStateException(
                    "marker() must follow at least one splineTo/lineTo in the same segment");
        }
        pendingMarkers.add(new Marker(completion, action));
        return this;
    }

    // ----- turning ---------------------------------------------------------------------

    /** Turns in place to an absolute field heading, in radians. */
    public RouteBuilder turnTo(double headingRadians) {
        flush();
        segments.add(new TurnSegment(headingRadians, true,
                defaults.getMaxAngularVelocity(), defaults.getMaxAngularAcceleration(),
                defaults.newTurnPid(), defaults.getHeadingTolerance(),
                defaults.getAngularVelocityTolerance(), defaults.getTurnSettleTime(),
                defaults.getTurnTimeout()));
        cursor = cursor.withHeading(MathUtil.normalizeAngle(headingRadians));
        return this;
    }

    public RouteBuilder turnToDegrees(double headingDegrees) {
        return turnTo(MathUtil.toRadians(headingDegrees));
    }

    /** Turns in place by a relative amount, in radians. Positive is counter-clockwise. */
    public RouteBuilder turn(double deltaRadians) {
        flush();
        segments.add(new TurnSegment(deltaRadians, false,
                defaults.getMaxAngularVelocity(), defaults.getMaxAngularAcceleration(),
                defaults.newTurnPid(), defaults.getHeadingTolerance(),
                defaults.getAngularVelocityTolerance(), defaults.getTurnSettleTime(),
                defaults.getTurnTimeout()));
        cursor = cursor.withHeading(MathUtil.normalizeAngle(cursor.getHeading() + deltaRadians));
        return this;
    }

    public RouteBuilder turnDegrees(double deltaDegrees) {
        return turn(MathUtil.toRadians(deltaDegrees));
    }

    // ----- stopping --------------------------------------------------------------------

    /**
     * Converges on the pose the route has reached and holds it. Exits as soon as it is inside
     * tolerance, so a generous timeout is free when the robot arrives cleanly.
     *
     * <p>Put one of these anywhere accuracy matters. Drive segments get the robot roughly
     * there fast; this is what gets it exactly there.
     */
    public RouteBuilder stopAndHold(double timeoutSeconds) {
        flush();
        segments.add(new HoldSegment(cursor, timeoutSeconds, defaults.getHoldSettleTime(), false));
        return this;
    }

    public RouteBuilder stopAndHold() {
        return stopAndHold(defaults.getHoldTimeout());
    }

    /** Holds a specific pose rather than the one the route reached. */
    public RouteBuilder holdPose(Pose pose, double timeoutSeconds) {
        flush();
        segments.add(new HoldSegment(pose, timeoutSeconds, defaults.getHoldSettleTime(), false));
        cursor = pose;
        return this;
    }

    /** Actively holds position for the full duration, whether or not it converges early. */
    public RouteBuilder holdFor(double seconds) {
        flush();
        segments.add(new HoldSegment(cursor, seconds, 0.0, true));
        return this;
    }

    /** Cuts drive power and waits. Use {@link #holdFor} if the robot must not drift. */
    public RouteBuilder waitSeconds(double seconds) {
        flush();
        segments.add(new WaitSegment(seconds));
        return this;
    }

    // ----- actions ---------------------------------------------------------------------

    /** Runs an action to completion, holding position, before continuing. */
    public RouteBuilder action(Action action) {
        return action(action, defaults.getActionTimeout());
    }

    public RouteBuilder action(Action action, double timeoutSeconds) {
        flush();
        segments.add(new ActionSegment(action, timeoutSeconds, "action"));
        return this;
    }

    // ----- build -----------------------------------------------------------------------

    public Route build() {
        flush();
        if (segments.isEmpty()) {
            throw new IllegalStateException("Route is empty");
        }
        return new Route(segments, start, cursor);
    }

    /** Planned pose after everything queued so far. Useful when composing routes. */
    public Pose currentPose() {
        return cursor;
    }

    public boolean isReversed() {
        return reversed;
    }

    // ----- internals -------------------------------------------------------------------

    private void ensurePending() {
        if (pendingPath == null) {
            pendingPath = SplinePath.builder(travelPose(cursor));
            pendingMarkers = new ArrayList<Marker>();
            pendingWaypoints = 0;
            pendingReversed = reversed;
        }
    }

    private void flush() {
        if (pendingPath == null) {
            return;
        }
        if (pendingWaypoints == 0) {
            pendingPath = null;
            pendingMarkers = null;
            return;
        }

        SplinePath path = pendingPath.build();
        String label = String.format("%s %.1f in, %d waypoint%s",
                pendingReversed ? "reverse" : "drive",
                path.length(),
                pendingWaypoints,
                pendingWaypoints == 1 ? "" : "s");

        segments.add(new DriveSegment(path, constraints.copy(), pendingReversed, pendingMarkers, label));

        pendingPath = null;
        pendingMarkers = null;
        pendingWaypoints = 0;
    }

    /** Robot heading to direction of travel, for the current direction. */
    private double travelHeading(double robotHeadingRadians) {
        return reversed ? MathUtil.normalizeAngle(robotHeadingRadians + Math.PI) : robotHeadingRadians;
    }

    /** Direction of travel back to robot heading. */
    private double robotHeading(double travelHeadingRadians) {
        return reversed ? MathUtil.normalizeAngle(travelHeadingRadians + Math.PI) : travelHeadingRadians;
    }

    private Pose travelPose(Pose robotPose) {
        return robotPose.withHeading(travelHeading(robotPose.getHeading()));
    }
}
