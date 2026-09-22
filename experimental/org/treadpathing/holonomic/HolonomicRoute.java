package org.treadpathing.holonomic;

import java.util.ArrayList;
import java.util.List;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.spline.SplinePath;
import org.treadpathing.trajectory.MotionProfile;

/**
 * A holonomic route: drive legs and holds, in order.
 *
 * <p>Written as its own small model rather than reusing {@code Route} and {@code Segment},
 * which are typed all the way down to a two-number chassis command. That is the finding this
 * experiment exists to produce, so it is left visible here rather than papered over.
 *
 * <p>What is missing compared to the tank builder is as interesting as what is here: there is
 * no {@code reversed()}, and no cusp. A mecanum robot changing direction does not have to stop
 * -- the path can turn a corner as sharply as traction allows while the nose does whatever it
 * was already doing -- so the one primitive the tank library needs most has nothing to do.
 */
public final class HolonomicRoute {

    /** One leg of a route: a drive along a trajectory, a turn in place, or a hold at a pose. */
    public static final class Leg {

        private final HolonomicTrajectory trajectory;
        private final Pose holdPose;
        private final double holdSeconds;
        private final MotionProfile turnProfile;
        private final double turnFrom;
        private final String label;

        private Leg(HolonomicTrajectory trajectory, Pose holdPose, double holdSeconds,
                    MotionProfile turnProfile, double turnFrom, String label) {
            this.trajectory = trajectory;
            this.holdPose = holdPose;
            this.holdSeconds = holdSeconds;
            this.turnProfile = turnProfile;
            this.turnFrom = turnFrom;
            this.label = label;
        }

        static Leg drive(HolonomicTrajectory trajectory, String label) {
            return new Leg(trajectory, null, 0.0, null, 0.0, label);
        }

        static Leg hold(Pose pose, double seconds, String label) {
            return new Leg(null, pose, seconds, null, 0.0, label);
        }

        static Leg turn(Pose pose, MotionProfile profile, double from, double settleSeconds,
                        String label) {
            // The profile is a reference, and the robot lags it: ending the leg the instant
            // the reference arrives leaves the real heading short, which on the first draft of
            // this experiment was 21 degrees. TurnSegment settles for the same reason.
            return new Leg(null, pose, profile.getDuration() + settleSeconds, profile, from, label);
        }

        public boolean isDrive() {
            return trajectory != null;
        }

        public boolean isTurn() {
            return turnProfile != null;
        }

        /** Where the nose should be at this instant of a turn in place. */
        public double headingAt(double elapsed) {
            if (turnProfile == null) {
                return holdPose.getHeading();
            }
            return MathUtil.normalizeAngle(turnFrom + turnProfile.position(elapsed));
        }

        public HolonomicTrajectory getTrajectory() {
            return trajectory;
        }

        public Pose getHoldPose() {
            return holdPose;
        }

        public double getHoldSeconds() {
            return holdSeconds;
        }

        public String describe() {
            return label;
        }
    }

    private final List<Leg> legs;
    private final Pose start;
    private final Pose end;

    HolonomicRoute(List<Leg> legs, Pose start, Pose end) {
        this.legs = legs;
        this.start = start;
        this.end = end;
    }

    public int size() {
        return legs.size();
    }

    public Leg get(int index) {
        return legs.get(index);
    }

    public Pose getStart() {
        return start;
    }

    public Pose getEnd() {
        return end;
    }

    public double plannedDuration() {
        double total = 0.0;
        for (int i = 0; i < legs.size(); i++) {
            Leg leg = legs.get(i);
            total += leg.isDrive() ? leg.getTrajectory().getDuration() : leg.getHoldSeconds();
        }
        return total;
    }

    /** Where the route ends up, including any turn in place at the end of it. */
    public Pose plannedEnd() {
        return end;
    }

    public String summary() {
        StringBuilder out = new StringBuilder(120);
        for (int i = 0; i < legs.size(); i++) {
            out.append(i + 1).append(". ").append(legs.get(i).describe()).append('\n');
        }
        out.append(String.format("%.2f s planned", plannedDuration()));
        return out.toString();
    }

    public static Builder builder(Pose start, HolonomicConstraints constraints,
                                  MecanumKinematics kinematics) {
        return new Builder(start, constraints, kinematics);
    }

    /**
     * Accumulates waypoints into a path the way {@code RouteBuilder} does, but with the nose
     * as a separate instruction rather than a consequence of the geometry.
     */
    public static final class Builder {

        private final List<Leg> legs = new ArrayList<Leg>();
        private final HolonomicConstraints constraints;
        private final MecanumKinematics kinematics;
        private final Pose start;

        private Pose cursor;
        private SplinePath.Builder pending;
        private int pendingWaypoints;
        private HeadingPlan plan;
        private double maxAngularAcceleration = 6.0;
        private double turnSettleSeconds = 0.5;

        private Builder(Pose start, HolonomicConstraints constraints, MecanumKinematics kinematics) {
            this.start = start;
            this.cursor = start;
            this.constraints = constraints;
            this.kinematics = kinematics;
            this.plan = HeadingPlans.hold(start.getHeading());
        }

        /**
         * Adds a waypoint. Only the position is used: on a holonomic drive the heading at a
         * waypoint is the heading plan's business, not the path's.
         */
        public Builder to(double x, double y) {
            return to(x, y, cursorTangentTo(x, y));
        }

        /**
         * Adds a waypoint with an explicit path tangent, for when the shape of the curve
         * matters -- the direction the robot <b>travels</b> through the point, still not the
         * direction it faces.
         */
        public Builder to(double x, double y, double tangentRadians) {
            if (pending == null) {
                pending = SplinePath.builder(new Pose(cursor.getX(), cursor.getY(),
                        cursorTangentTo(x, y)));
                pendingWaypoints = 0;
            }
            pending.to(new Pose(x, y, tangentRadians));
            pendingWaypoints++;
            cursor = new Pose(x, y, cursor.getHeading());
            return this;
        }

        /** Points the nose at a fixed angle for everything that follows. */
        public Builder faceAngle(double headingRadians) {
            return withPlan(HeadingPlans.hold(headingRadians));
        }

        /** Points the nose along the direction of travel, the way a tank drive must. */
        public Builder faceTangent() {
            return withPlan(HeadingPlans.tangent());
        }

        /** Turns the nose steadily to this angle across the leg that follows. */
        public Builder turnAcross(double headingRadians) {
            return withPlan(HeadingPlans.interpolate(cursor.getHeading(), headingRadians));
        }

        public Builder withPlan(HeadingPlan newPlan) {
            // A change of plan closes the current leg, because the plan is what the leg's
            // profile was solved against. Unlike a tank cusp this costs nothing but a sample:
            // the robot is not required to stop at the seam.
            flush();
            plan = newPlan;
            return this;
        }

        /**
         * Turns on the spot to face a new angle.
         *
         * <p>The one primitive a holonomic route needs that a heading plan cannot express: a
         * plan says what the nose does <b>while travelling</b>, and this is what it does when
         * there is nowhere to travel. Profiled, so the turn is not a step input at the motors.
         */
        public Builder turnTo(double headingRadians) {
            flush();
            double delta = MathUtil.angleDelta(cursor.getHeading(), headingRadians);
            MotionProfile profile = new MotionProfile(delta,
                    constraints.getMaxAngularVelocity(), maxAngularAcceleration);
            legs.add(Leg.turn(cursor, profile, cursor.getHeading(), turnSettleSeconds,
                    String.format("turn to %.0f deg", Math.toDegrees(headingRadians))));
            cursor = new Pose(cursor.getX(), cursor.getY(), headingRadians);
            plan = HeadingPlans.hold(headingRadians);
            return this;
        }

        /** How long a turn holds its target after the profile ends, letting the robot catch up. */
        public Builder turnSettleSeconds(double seconds) {
            this.turnSettleSeconds = seconds;
            return this;
        }

        /** Angular acceleration for turns in place, radians per second squared. */
        public Builder maxAngularAcceleration(double radiansPerSecondSquared) {
            this.maxAngularAcceleration = radiansPerSecondSquared;
            return this;
        }

        public Builder holdFor(double seconds) {
            flush();
            legs.add(Leg.hold(cursor, seconds,
                    String.format("hold %.1f s at %.0f, %.0f", seconds, cursor.getX(), cursor.getY())));
            return this;
        }

        public HolonomicRoute build() {
            flush();
            if (legs.isEmpty()) {
                throw new IllegalStateException("Route is empty");
            }
            return new HolonomicRoute(legs, start, cursor);
        }

        private void flush() {
            if (pending == null || pendingWaypoints == 0) {
                pending = null;
                return;
            }
            SplinePath path = pending.build();
            HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generate(
                    path, plan, constraints, kinematics);
            legs.add(Leg.drive(trajectory, String.format("drive %.1f in, %s",
                    path.length(), plan.describe())));
            cursor = new Pose(cursor.getX(), cursor.getY(),
                    plan.headingAt(path, path.length()));
            pending = null;
            pendingWaypoints = 0;
        }

        private double cursorTangentTo(double x, double y) {
            double dx = x - cursor.getX();
            double dy = y - cursor.getY();
            return Math.abs(dx) < MathUtil.EPSILON && Math.abs(dy) < MathUtil.EPSILON
                    ? cursor.getHeading() : Math.atan2(dy, dx);
        }
    }
}
