package org.treadpathing.holonomic;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.geometry.Twist2;
import org.treadpathing.localization.HeadingFuser;
import org.treadpathing.localization.Localizer;
import org.treadpathing.localization.VelocityEstimator;

/**
 * Odometry from the four drive encoders and the hub IMU. No pods, no Pinpoint, nothing to
 * mount.
 *
 * <h3>Read this before you rely on it</h3>
 *
 * A mecanum wheel moves the robot with the part of the roller that grips and wastes the rest.
 * Forward travel grips well and measures well. Sideways travel is the rollers doing what they
 * are shaped to do, and they slip: how much depends on the floor, the wheel wear and how much
 * weight is on each corner, which is what {@code lateralMultiplier} is estimating and why it
 * is a measured number rather than a computed one. So this is <b>good at forwards, poor at
 * strafing, and worst when it matters most</b>.
 *
 * <p>The tank library's {@code DriveEncoderLocalizer} carries a similar warning, and this one
 * is worse: a tank drive only scrubs while turning, whereas a mecanum scrubs whenever it is
 * not driving straight. Treat it as bring-up odometry — enough to prove the drivetrain, the
 * constants and a route before the pods arrive — and expect a strafing auto to drift.
 *
 * <h3>The kinematics</h3>
 *
 * The forward map, inverted from the wheel speeds in {@link HolonomicSpeeds}, with the wheels
 * in front-left, front-right, back-left, back-right order:
 *
 * <pre>
 * forward = ( fl + fr + bl + br) / 4
 * lateral = (-fl + fr + bl - br) / 4 / lateralMultiplier
 * </pre>
 *
 * <p>Heading comes from the IMU and never from the wheels. The rotation term of the inverse is
 * {@code (-fl + fr - bl + br) / 4 / turnRadius}, and it is the term the slip corrupts most: a
 * robot pushed sideways reports a turn it never made. The IMU costs one read and is right.
 */
public final class MecanumEncoderLocalizer implements Localizer {

    private final MecanumDrive drive;
    private final HeadingFuser headingFuser;
    private final MecanumKinematics kinematics;

    private final VelocityEstimator forwardVelocity = new VelocityEstimator();
    private final VelocityEstimator lateralVelocity = new VelocityEstimator();

    private Pose pose = new Pose(0.0, 0.0, 0.0);
    private double[] lastWheels;
    private double forwardTravel;
    private double lateralTravel;
    private double lastHeading;
    private double elapsed;
    private boolean primed;

    public MecanumEncoderLocalizer(MecanumDrive drive, HeadingFuser headingFuser) {
        this.drive = drive;
        this.headingFuser = headingFuser;
        this.kinematics = drive.getKinematics();
        // This localizer feeds the fuser zero rotation between IMU reads, on purpose -- see
        // update(). With any decimation above 1 that would freeze the heading between reads,
        // so every pose update in between would be integrated along a stale heading. Reading
        // every loop is the only setting that is correct here, so it is not left to the caller.
        headingFuser.setDecimation(1);
    }

    @Override
    public void update(double dt) {
        double[] wheels = drive.getWheelPositionsInches();
        elapsed += dt;

        if (!primed) {
            lastWheels = wheels;
            // Nothing to feed the fuser yet: the wheels cannot say anything about heading.
            headingFuser.update(0.0, dt);
            lastHeading = headingFuser.getHeading();
            pose = pose.withHeading(lastHeading);
            primed = true;
            return;
        }

        double fl = wheels[0] - lastWheels[0];
        double fr = wheels[1] - lastWheels[1];
        double bl = wheels[2] - lastWheels[2];
        double br = wheels[3] - lastWheels[3];
        lastWheels = wheels;

        double[] motion = kinematics.forward(fl, fr, bl, br);
        double forward = motion[0];
        double lateral = motion[1];

        // Zero, not the wheels' own estimate. The rotation term is the one slip corrupts
        // worst, and feeding a bad estimate in would let the fuser coast on it between reads.
        headingFuser.update(0.0, dt);

        double heading = headingFuser.getHeading();
        double deltaHeading = MathUtil.angleDelta(lastHeading, heading);
        lastHeading = heading;

        pose = pose.exp(new Twist2(forward, lateral, deltaHeading));
        // Trust the IMU absolutely rather than the integrated delta, so nothing accumulates.
        pose = pose.withHeading(heading);

        forwardTravel += forward;
        lateralTravel += lateral;
        forwardVelocity.add(elapsed, forwardTravel);
        lateralVelocity.add(elapsed, lateralTravel);
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
        lastWheels = drive.getWheelPositionsInches();
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
        return "drive encoders + IMU (bring-up only; strafes drift)";
    }
}
