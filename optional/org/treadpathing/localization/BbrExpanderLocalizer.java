package org.treadpathing.localization;

import com.buildingblockrobotics.expander.BBRDigitalExpander;
import com.buildingblockrobotics.expander.Pose2D;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/**
 * Localizer for the BotBuilders Robotics Digital Expander.
 *
 * <p>The board fuses its own gyro with two dead wheels and hands back a finished field pose in
 * millimetres, so this is a thin adapter rather than a driver: it converts units, adopts the
 * library's inches-and-radians convention, and derives the velocities the follower needs.
 *
 * <h3>This file is not part of the default drop</h3>
 *
 * It lives in {@code optional/} because it depends on BotBuilders' own driver, which is two
 * more source files you have to paste. Everything in {@code src/} compiles against the FTC SDK
 * and nothing else, and putting this beside it would mean every team had to paste a driver for
 * a board most of them do not own, or watch Build Everything fail. Copy this file only if you
 * have the board.
 *
 * <h3>Installing it</h3>
 *
 * <ol>
 *   <li>Paste BotBuilders' {@code com/buildingblockrobotics/expander/} folder
 *       ({@code BBRDigitalExpander.java} and {@code BBRRegMap.java}) into the OnBotJava tree,
 *       as a sibling of {@code org/}. See expander.buildingblockrobotics.com.
 *   <li>Paste this file into {@code org/treadpathing/localization/}.
 *   <li>Add the board to the robot configuration under I2C Devices as
 *       <b>BBR Digital Expander</b>.
 *   <li>Point the factory at it, in {@code Constants.localizerFactory()}:
 *       <pre>return new BbrExpanderLocalizer(hardwareMap, "expander");</pre>
 * </ol>
 *
 * <h3>Before it will localize</h3>
 *
 * The pods, their ticks per millimetre and the tracking-point offsets are configured on the
 * board itself and saved to its flash, not here -- see the odometry guide. Calibration wants
 * the robot stationary for a second or two, which is why the constructor waits for the
 * localizer to report ready rather than trusting the first reading. That is the same trap the
 * IMU heading fuser has: a reading taken too early looks like a perfectly innocent zero.
 */
public final class BbrExpanderLocalizer implements Localizer {

    /** The board speaks millimetres; everything here is inches. */
    private static final double MM_PER_INCH = 25.4;

    /** How long to let the board finish calibrating before giving up on it. */
    private static final int READY_TIMEOUT_MS = 3000;

    private final BBRDigitalExpander device;

    // The board reports a pose, not a velocity, so the velocities are differentiated here.
    private final VelocityEstimator forwardVelocity = new VelocityEstimator();
    private final VelocityEstimator lateralVelocity = new VelocityEstimator();
    private final VelocityEstimator headingVelocity = new VelocityEstimator();

    private Pose pose = new Pose(0.0, 0.0, 0.0);
    private double time;
    private double alongTrack;
    private double crossTrack;
    private double unwrappedHeading;
    private boolean primed;
    private boolean ready;

    public BbrExpanderLocalizer(HardwareMap hardwareMap, String name) {
        this(hardwareMap.get(BBRDigitalExpander.class, name));
    }

    public BbrExpanderLocalizer(BBRDigitalExpander device) {
        this.device = device;
        this.ready = device.waitForLocalizerReady(READY_TIMEOUT_MS);
    }

    public BBRDigitalExpander getDevice() {
        return device;
    }

    @Override
    public void update(double dt) {
        time += dt;

        Pose2D reading = device.getPose();
        if (reading == null) {
            return;
        }

        double x = reading.xMm / MM_PER_INCH;
        double y = reading.yMm / MM_PER_INCH;
        double heading = MathUtil.normalizeAngle(reading.headingRad);
        pose = new Pose(x, y, heading);

        // Differentiating a wrapped angle puts a full turn of nonsense into the estimate every
        // time it crosses pi, so accumulate the deltas instead and differentiate that.
        if (primed) {
            unwrappedHeading += MathUtil.angleDelta(
                    MathUtil.normalizeAngle(unwrappedHeading), heading);
        } else {
            unwrappedHeading = heading;
            primed = true;
        }

        // Field displacement resolved into the robot frame, because the follower asks for
        // forward and lateral speed rather than dx and dy.
        alongTrack = x * Math.cos(heading) + y * Math.sin(heading);
        crossTrack = -x * Math.sin(heading) + y * Math.cos(heading);

        forwardVelocity.add(time, alongTrack);
        lateralVelocity.add(time, crossTrack);
        headingVelocity.add(time, unwrappedHeading);
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public void setPose(Pose newPose) {
        device.setPose(new Pose2D(newPose.getX() * MM_PER_INCH,
                                  newPose.getY() * MM_PER_INCH,
                                  newPose.getHeading()));
        pose = newPose;
        // The old samples describe a robot that was somewhere else.
        forwardVelocity.reset();
        lateralVelocity.reset();
        headingVelocity.reset();
        primed = false;
    }

    @Override
    public double getForwardVelocity() {
        return forwardVelocity.getVelocity();
    }

    @Override
    public double getLateralVelocity() {
        return lateralVelocity.getVelocity();
    }

    @Override
    public double getAngularVelocity() {
        return headingVelocity.getVelocity();
    }

    @Override
    public String status() {
        if (!ready) {
            return "BBR expander: localizer never reported ready";
        }
        return device.isDataFresh() ? "BBR expander" : "BBR expander: stale data";
    }

    /** False if the board never finished calibrating, or has stopped producing new readings. */
    public boolean isHealthy() {
        return ready && device.isDataFresh();
    }
}
