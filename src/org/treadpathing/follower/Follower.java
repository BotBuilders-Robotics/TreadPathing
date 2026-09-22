package org.treadpathing.follower;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.treadpathing.control.ChassisSpeeds;
import org.treadpathing.control.PoseHoldController;
import org.treadpathing.control.TrajectoryController;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.hardware.BulkReader;
import org.treadpathing.hardware.TankDrive;
import org.treadpathing.localization.Localizer;
import org.treadpathing.localization.LocalizerFactory;
import org.treadpathing.route.Action;
import org.treadpathing.route.Route;
import org.treadpathing.route.RouteBuilder;
import org.treadpathing.route.Segment;
import org.treadpathing.route.SegmentHost;
import org.treadpathing.util.Clock;
import org.treadpathing.util.SystemClock;

/**
 * The one object a team touches. Owns the drivetrain, the localizer and the control loop.
 *
 * <pre>
 * Follower follower = new Follower(hardwareMap, Constants.follower(), Constants.localizer(hardwareMap));
 * follower.setPose(new Pose(9.0, 60.0, 0.0));
 *
 * Route route = follower.route()
 *         .splineTo(new Pose(34.0, 60.0, 0.0))
 *         .stopAndHold(1.0)
 *         .build();
 *
 * waitForStart();
 * follower.follow(route);
 * while (opModeIsActive() &amp;&amp; follower.isBusy()) {
 *     follower.update();
 *     telemetry.addLine(follower.telemetry());
 *     telemetry.update();
 * }
 * </pre>
 *
 * <p>{@link #update()} does exactly one pass: clear the bulk cache, read the localizer, poll
 * background actions, let the active segment compute a command, write it to the motors. No
 * threads — the SDK holds a lock per USB device, so moving hardware calls onto a worker only
 * makes them interleave and get slower.
 */
public final class Follower implements SegmentHost {

    private final TankDrive drive;
    private final Localizer localizer;
    private final BulkReader bulkReader;
    private final FollowerConstants constants;
    private final TrajectoryController trajectoryController;
    private final PoseHoldController poseHoldController;
    private final Clock clock;

    private final List<Action> background = new ArrayList<Action>();

    private Route route;
    private int segmentIndex;
    private Segment activeSegment;
    private boolean busy;

    private Pose pose = new Pose(0.0, 0.0, 0.0);
    private double forwardVelocity;
    private double angularVelocity;
    private double lastTimestamp = -1.0;
    private double loopPeriod = 0.02;
    private double worstLoopPeriod;

    public Follower(HardwareMap hardwareMap, FollowerConstants constants, Localizer localizer) {
        this(hardwareMap, constants, localizer, new SystemClock());
    }

    public Follower(HardwareMap hardwareMap, FollowerConstants constants, Localizer localizer,
                    Clock clock) {
        this.constants = constants;
        this.clock = clock;
        this.bulkReader = new BulkReader(hardwareMap);
        this.drive = new TankDrive(hardwareMap, constants.getDriveConstants());
        this.localizer = localizer;
        this.trajectoryController = constants.buildTrajectoryController();
        this.poseHoldController = constants.buildPoseHoldController();
    }

    /**
     * Builds the drivetrain first, then hands it to the factory. Use this whenever the
     * localizer needs to read the drive motors, which {@code DriveEncoderLocalizer} does.
     */
    public Follower(HardwareMap hardwareMap, FollowerConstants constants,
                    LocalizerFactory localizerFactory) {
        this(hardwareMap, constants, localizerFactory, new SystemClock());
    }

    public Follower(HardwareMap hardwareMap, FollowerConstants constants,
                    LocalizerFactory localizerFactory, Clock clock) {
        this.constants = constants;
        this.clock = clock;
        this.bulkReader = new BulkReader(hardwareMap);
        this.drive = new TankDrive(hardwareMap, constants.getDriveConstants());
        this.localizer = localizerFactory.create(hardwareMap, this.drive);
        this.trajectoryController = constants.buildTrajectoryController();
        this.poseHoldController = constants.buildPoseHoldController();
    }

    // ----- setup ------------------------------------------------------------------------

    public TankDrive getDrive() {
        return drive;
    }

    public Localizer getLocalizer() {
        return localizer;
    }

    public BulkReader getBulkReader() {
        return bulkReader;
    }

    public FollowerConstants getConstants() {
        return constants;
    }

    /** Declares where the robot is. Call it in init, before building a route. */
    public void setPose(Pose newPose) {
        localizer.setPose(newPose);
        pose = newPose;
    }

    /** A builder seeded at the robot's current pose, with the team's default limits. */
    public RouteBuilder route() {
        return new RouteBuilder(pose, constants.getRouteDefaults());
    }

    /** A builder seeded at an explicit pose, for routes planned before the robot is placed. */
    public RouteBuilder routeFrom(Pose start) {
        return new RouteBuilder(start, constants.getRouteDefaults());
    }

    // ----- running ----------------------------------------------------------------------

    public void follow(Route newRoute) {
        this.route = newRoute;
        this.segmentIndex = 0;
        this.activeSegment = null;
        this.busy = newRoute.size() > 0;
        this.background.clear();
    }

    public boolean isBusy() {
        return busy;
    }

    public int getSegmentIndex() {
        return segmentIndex;
    }

    /** The segment currently running, or null when idle. Used by the tuning OpModes. */
    public Segment getActiveSegment() {
        return activeSegment;
    }

    public Route getRoute() {
        return route;
    }

    /** Abandons the route and cuts drive power. */
    public void breakFollowing() {
        busy = false;
        route = null;
        activeSegment = null;
        drive.stop();
    }

    /**
     * One control pass. Call it as fast as the loop allows; everything is driven by the
     * measured period, so a jittery loop degrades gracefully rather than misbehaving.
     */
    public void update() {
        bulkReader.clearCache();

        double now = clock.seconds();
        if (lastTimestamp < 0.0) {
            loopPeriod = constants.getMinLoopPeriod();
        } else {
            loopPeriod = MathUtil.clamp(now - lastTimestamp,
                    constants.getMinLoopPeriod(), constants.getMaxLoopPeriod());
            worstLoopPeriod = Math.max(worstLoopPeriod, now - lastTimestamp);
        }
        lastTimestamp = now;

        localizer.update(loopPeriod);
        pose = localizer.getPose();
        forwardVelocity = localizer.getForwardVelocity();
        angularVelocity = localizer.getAngularVelocity();

        runBackgroundActions();

        if (!busy || route == null) {
            drive.stop();
            return;
        }

        if (activeSegment == null) {
            activeSegment = route.get(segmentIndex);
            activeSegment.init(this);
        }

        if (activeSegment.update(this)) {
            segmentIndex++;
            if (segmentIndex >= route.size()) {
                busy = false;
                activeSegment = null;
                drive.stop();
            } else {
                activeSegment = route.get(segmentIndex);
                activeSegment.init(this);
            }
        }
    }

    private void runBackgroundActions() {
        for (int i = background.size() - 1; i >= 0; i--) {
            if (background.get(i).run()) {
                background.remove(i);
            }
        }
    }

    // ----- SegmentHost ------------------------------------------------------------------

    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public double getForwardVelocity() {
        return forwardVelocity;
    }

    @Override
    public double getAngularVelocity() {
        return angularVelocity;
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        drive.setChassisSpeeds(speeds, 0.0);
    }

    @Override
    public void drive(ChassisSpeeds speeds, double linearAcceleration) {
        drive.setChassisSpeeds(speeds, linearAcceleration);
    }

    @Override
    public void stopDrive() {
        drive.stop();
    }

    @Override
    public TrajectoryController getTrajectoryController() {
        return trajectoryController;
    }

    @Override
    public PoseHoldController getPoseHoldController() {
        return poseHoldController;
    }

    @Override
    public double time() {
        return clock.seconds();
    }

    @Override
    public double dt() {
        return loopPeriod;
    }

    @Override
    public void submitBackgroundAction(Action action) {
        action.start();
        background.add(action);
    }

    // ----- telemetry --------------------------------------------------------------------

    public double getLoopPeriod() {
        return loopPeriod;
    }

    public double getLoopHz() {
        return loopPeriod > 1e-6 ? 1.0 / loopPeriod : 0.0;
    }

    /** Worst loop period seen since the follower was built. Watch this, not the average. */
    public double getWorstLoopPeriod() {
        return worstLoopPeriod;
    }

    /** Multi-line status for telemetry. */
    public String telemetry() {
        StringBuilder out = new StringBuilder(160);
        out.append(pose).append('\n');
        out.append(String.format("v %.1f in/s   w %.2f rad/s\n", forwardVelocity, angularVelocity));
        out.append(String.format("loop %.0f Hz (worst %.0f ms)\n",
                getLoopHz(), worstLoopPeriod * 1000.0));
        out.append(localizer.status()).append('\n');
        out.append(trajectoryController.name());
        if (busy && route != null) {
            out.append("  seg ").append(segmentIndex).append('/').append(route.size())
                    .append(": ").append(route.get(segmentIndex).describe());
        } else {
            out.append("  idle");
        }
        return out.toString();
    }
}
