package org.treadpathing.route;

import java.util.ArrayList;
import java.util.List;

import org.treadpathing.control.ChassisSpeeds;
import org.treadpathing.geometry.Pose;
import org.treadpathing.spline.SplinePath;
import org.treadpathing.trajectory.Trajectory;
import org.treadpathing.trajectory.TrajectoryConstraints;
import org.treadpathing.trajectory.TrajectoryGenerator;
import org.treadpathing.trajectory.TrajectorySample;

/**
 * Follows one trajectory. The bread and butter of a route.
 *
 * <p>The trajectory is generated in the constructor, which runs while {@code RouteBuilder}
 * builds the route — during the OpMode's init, never mid-match. Generation costs a few
 * milliseconds and allocates; doing it in a control loop would blow the loop budget and
 * invite a garbage collection pause at the worst possible moment.
 *
 * <p>The geometry is planned, not re-planned. If the robot enters this segment slightly off
 * the planned start pose, the trajectory is not shifted to match — the controller pulls the
 * robot onto the planned path instead. That keeps the path the robot drives identical to the
 * one drawn in the visualizer, which matters more than shaving the first inch of error.
 */
public final class DriveSegment extends Segment {

    private final Trajectory trajectory;
    private final SplinePath path;
    private final boolean reversed;
    private final Marker[] markers;
    private final boolean[] fired;
    private final String label;

    private double startTime;
    private TrajectorySample lastReference;

    public DriveSegment(SplinePath path, TrajectoryConstraints constraints, boolean reversed,
                        List<Marker> markers, String label) {
        this.path = path;
        this.reversed = reversed;
        this.trajectory = TrajectoryGenerator.generate(path, constraints, reversed);
        List<Marker> copy = markers == null ? new ArrayList<Marker>() : markers;
        this.markers = copy.toArray(new Marker[copy.size()]);
        this.fired = new boolean[this.markers.length];
        this.label = label;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public SplinePath getPath() {
        return path;
    }

    public boolean isReversed() {
        return reversed;
    }

    @Override
    public void init(SegmentHost host) {
        startTime = host.time();
        host.getTrajectoryController().reset();
        for (int i = 0; i < fired.length; i++) {
            fired[i] = false;
        }
    }

    @Override
    public boolean update(SegmentHost host) {
        double elapsed = host.time() - startTime;
        TrajectorySample reference = trajectory.sample(elapsed);
        lastReference = reference;

        ChassisSpeeds command = host.getTrajectoryController().calculate(host.getPose(), reference);
        host.drive(command, reference.getAcceleration());

        double completion = trajectory.completionAt(elapsed);
        for (int i = 0; i < markers.length; i++) {
            if (!fired[i] && completion >= markers[i].getCompletion()) {
                fired[i] = true;
                host.submitBackgroundAction(markers[i].getAction());
            }
        }

        return elapsed >= trajectory.getDuration();
    }

    /**
     * The reference the follower was tracking on the last loop, or null before the first one.
     * This is what a datalog needs in order to plot planned against actual.
     */
    public TrajectorySample getLastReference() {
        return lastReference;
    }

    @Override
    public Pose plannedEndPose(Pose plannedStartPose) {
        return trajectory.endPose();
    }

    @Override
    public String describe() {
        return label;
    }
}
