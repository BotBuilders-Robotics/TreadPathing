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

        /**
         * How fast the nose should be turning at this instant of a turn in place, radians per
         * second. Zero for a hold, and after the profile ends.
         *
         * <p>Handed to the pose hold as feedforward. Without it the hold chases the profile on
         * proportional gain alone, which only turns while it is behind -- so the robot lags the
         * whole way round and the settle window spends its time catching up.
         */
        public double omegaAt(double elapsed) {
            return turnProfile == null ? 0.0 : turnProfile.velocity(elapsed);
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

        /**
         * Pieces driven back to back without a stop, waiting to be profiled as one trajectory.
         * The run ends -- and the robot stops -- only at a hold, a turn or the end of the route.
         */
        private final List<SplinePath> runPaths = new ArrayList<SplinePath>();
        private final List<HeadingPlan> runPlans = new ArrayList<HeadingPlan>();
        private double runEndTangent;

        private Pose cursor;
        private SplinePath.Builder pending;
        private int pendingWaypoints;
        private HeadingPlan plan;
        /**
         * Largest heading step a new plan may start with before the builder turns on the spot
         * to close it. Big enough to ignore rounding at a seam, small enough that the
         * controller absorbs what is left without saturating.
         */
        private static final double HEADING_STEP_TOLERANCE = Math.toRadians(1.0);

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
                // Carrying on from a piece that has not stopped, the path has to leave in the
                // direction the robot is already travelling. A kink here would be a step in
                // the velocity vector, which no profile can drive through.
                double startTangent = runPaths.isEmpty() ? cursorTangentTo(x, y) : runEndTangent;
                pending = SplinePath.builder(new Pose(cursor.getX(), cursor.getY(), startTangent));
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

        /**
         * Points the nose at a fixed offset from the direction of travel.
         *
         * <p>An offset of pi is the holonomic version of {@code reversed()}: the robot drives
         * the path backwards. Unlike the tank version there is no cusp, because the nose was
         * never what decided the direction of travel. The nose still has to get round,
         * though: switching to this from a nose-first plan is a turn on the spot at the seam.
         */
        public Builder faceTangent(double offsetRadians) {
            return withPlan(HeadingPlans.tangent(offsetRadians));
        }

        /** Turns the nose steadily to this angle across the leg that follows. */
        public Builder turnAcross(double headingRadians) {
            // Flush first: the waypoints already given may end on a different heading from
            // the one the cursor holds until they are closed off, and the turn has to start
            // from where the nose will actually be.
            flush();
            return withPlan(HeadingPlans.interpolate(cursor.getHeading(), headingRadians));
        }

        /**
         * Changes what the nose does from here on.
         *
         * <p>Unlike a tank cusp this does not stop the robot: the new piece is profiled
         * together with the one before it, so the speed carries straight through the seam. The
         * path leaves the seam in the direction it arrived, for the same reason.
         *
         * <p>The one thing it cannot carry through is a jump in heading. If the new plan wants
         * the nose somewhere other than where the last one left it, the robot stops there and
         * turns on the spot first -- see {@link #flush}.
         */
        public Builder withPlan(HeadingPlan newPlan) {
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
            endRun();
            addTurn(cursor.getX(), cursor.getY(), headingRadians,
                    String.format("turn to %.0f deg", Math.toDegrees(headingRadians)));
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
            endRun();
            legs.add(Leg.hold(cursor, seconds,
                    String.format("hold %.1f s at %.0f, %.0f", seconds, cursor.getX(), cursor.getY())));
            return this;
        }

        public HolonomicRoute build() {
            flush();
            endRun();
            if (legs.isEmpty()) {
                throw new IllegalStateException("Route is empty");
            }
            return new HolonomicRoute(legs, start, cursor);
        }

        /**
         * Closes the waypoints given so far into a piece of the current run.
         *
         * <p>This is also where a heading step is caught. A plan is free to ask for any nose
         * angle at the start of its path -- {@code faceAngle(90)} from a robot facing 0 is the
         * obvious one -- but a trajectory that begins 90 degrees away from the robot hands the
         * controller an error the planner never budgeted a wheel for. It spins at whatever
         * the heading gain asks, the command saturates, and translation is scaled down with
         * it. So the run stops there and a profiled turn in place closes the gap first.
         */
        private void flush() {
            if (pending == null || pendingWaypoints == 0) {
                pending = null;
                return;
            }
            SplinePath path = pending.build();
            pending = null;
            pendingWaypoints = 0;

            double startHeading = plan.headingAt(path, 0.0);
            if (Math.abs(MathUtil.angleDelta(cursor.getHeading(), startHeading))
                    > HEADING_STEP_TOLERANCE) {
                endRun();
                Pose from = path.poseAt(0.0);
                addTurn(from.getX(), from.getY(), startHeading,
                        String.format("turn to %.0f deg to start the next leg",
                                Math.toDegrees(startHeading)));
            }

            runPaths.add(path);
            runPlans.add(plan);
            runEndTangent = path.poseAt(path.length()).getHeading();
            cursor = new Pose(cursor.getX(), cursor.getY(),
                    plan.headingAt(path, path.length()));
        }

        /** Profiles the current run as one trajectory, ending at rest. */
        private void endRun() {
            if (runPaths.isEmpty()) {
                return;
            }
            HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generate(
                    new ArrayList<SplinePath>(runPaths), new ArrayList<HeadingPlan>(runPlans),
                    constraints, kinematics, HolonomicTrajectoryGenerator.DEFAULT_SAMPLE_SPACING);
            legs.add(Leg.drive(trajectory, String.format("drive %.1f in, %s",
                    trajectory.getLength(), trajectory.getHeadingPlan())));
            runPaths.clear();
            runPlans.clear();
        }

        private void addTurn(double x, double y, double headingRadians, String label) {
            double delta = MathUtil.angleDelta(cursor.getHeading(), headingRadians);
            MotionProfile profile = new MotionProfile(delta,
                    constraints.getMaxAngularVelocity(), maxAngularAcceleration);
            legs.add(Leg.turn(new Pose(x, y, cursor.getHeading()), profile, cursor.getHeading(),
                    turnSettleSeconds, label));
            cursor = new Pose(cursor.getX(), cursor.getY(), headingRadians);
        }

        private double cursorTangentTo(double x, double y) {
            double dx = x - cursor.getX();
            double dy = y - cursor.getY();
            return Math.abs(dx) < MathUtil.EPSILON && Math.abs(dy) < MathUtil.EPSILON
                    ? cursor.getHeading() : Math.atan2(dy, dx);
        }
    }
}
