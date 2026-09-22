package org.treadpathing.localization;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.geometry.Twist2;
import org.treadpathing.hardware.Encoder;

/**
 * Odometry from two unpowered dead wheels plus the IMU. The serious option.
 *
 * <p>Unpowered pods do not scrub, so translation stays honest through a turn — which is
 * exactly where drive encoders fall apart. Two pods plus an IMU is all a differential drive
 * needs: the parallel pod sees forward motion, the perpendicular pod sees the sideways
 * scrubbing the chassis does anyway, and the IMU supplies heading.
 *
 * <h3>Pod offsets</h3>
 *
 * Both offsets are measured from the robot's centre of rotation, in inches, in the robot's
 * frame: <b>x forward, y to the left</b>. A pod does not measure the chassis centre's motion,
 * it measures its own, so the offsets correct for the extra arc a pod sweeps while the robot
 * turns:
 *
 * <pre>
 *   forward = parallelReading + parallelPodY * dTheta
 *   lateral = perpendicularReading - perpendicularPodX * dTheta
 * </pre>
 *
 * <p>Get a sign wrong here and the robot's position will look fine driving straight and drift
 * badly during turns. The OffsetTest OpMode exists to catch exactly that.
 */
public final class TwoWheelLocalizer implements Localizer {

    private final Encoder parallel;
    private final Encoder perpendicular;
    private final double parallelPodY;
    private final double perpendicularPodX;
    private final HeadingFuser headingFuser;

    private final VelocityEstimator forwardVelocity = new VelocityEstimator();
    private final VelocityEstimator lateralVelocity = new VelocityEstimator();

    private Pose pose = new Pose(0.0, 0.0, 0.0);
    private double cumulativeForward;
    private double cumulativeLateral;
    private double lastParallel;
    private double lastPerpendicular;
    private double lastHeading;
    private double elapsed;
    private boolean primed;

    /**
     * @param parallel          pod rolling in the robot's forward direction
     * @param perpendicular     pod rolling sideways
     * @param parallelPodY      the parallel pod's offset to the robot's left, inches
     * @param perpendicularPodX the perpendicular pod's offset forward of centre, inches
     */
    public TwoWheelLocalizer(Encoder parallel, Encoder perpendicular,
                             double parallelPodY, double perpendicularPodX,
                             HeadingFuser headingFuser) {
        this.parallel = parallel;
        this.perpendicular = perpendicular;
        this.parallelPodY = parallelPodY;
        this.perpendicularPodX = perpendicularPodX;
        this.headingFuser = headingFuser;
    }

    @Override
    public void update(double dt) {
        double par = parallel.getPositionInches();
        double perp = perpendicular.getPositionInches();
        elapsed += dt;

        // Two pods cannot observe rotation on their own, so the fuser gets nothing to coast
        // on. Keep IMU decimation at 1 unless you have measured that you can afford otherwise.
        headingFuser.update(0.0, dt);
        double heading = headingFuser.getHeading();

        if (!primed) {
            lastParallel = par;
            lastPerpendicular = perp;
            lastHeading = heading;
            pose = pose.withHeading(heading);
            primed = true;
            return;
        }

        double deltaPar = par - lastParallel;
        double deltaPerp = perp - lastPerpendicular;
        double deltaHeading = MathUtil.angleDelta(lastHeading, heading);
        lastParallel = par;
        lastPerpendicular = perp;
        lastHeading = heading;

        double forward = deltaPar + parallelPodY * deltaHeading;
        double lateral = deltaPerp - perpendicularPodX * deltaHeading;

        cumulativeForward += forward;
        cumulativeLateral += lateral;
        forwardVelocity.add(elapsed, cumulativeForward);
        lateralVelocity.add(elapsed, cumulativeLateral);

        pose = pose.exp(new Twist2(forward, lateral, deltaHeading));
        pose = pose.withHeading(heading);
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public void setPose(Pose newPose) {
        pose = newPose;
        headingFuser.setHeading(newPose.getHeading());
        lastHeading = newPose.getHeading();
        lastParallel = parallel.getPositionInches();
        lastPerpendicular = perpendicular.getPositionInches();
        forwardVelocity.reset();
        lateralVelocity.reset();
        primed = true;
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
        return headingFuser.getAngularVelocity();
    }

    @Override
    public String status() {
        return "two dead wheels + IMU";
    }
}
