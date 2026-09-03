package org.treadpathing.localization;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.geometry.Twist2;
import org.treadpathing.hardware.TankDrive;

/**
 * Odometry from the drive motors' own encoders, with IMU heading.
 *
 * <p>The cheapest thing that works, and the right way to bring the library up on a robot that
 * has no dead wheels yet. Be clear-eyed about it: every turn scrubs the drive wheels, so
 * translation accumulates error exactly where the robot is working hardest. It is fine on
 * straights and it will get an auto roughly where it is going. It will not win anything.
 *
 * <p>Because it has two parallel measurements, it <i>can</i> estimate heading rate from the
 * wheels, which lets {@link HeadingFuser} coast between IMU reads and buys back loop time.
 * That estimate is only ever trusted for a few tens of milliseconds before the IMU corrects
 * it.
 */
public final class DriveEncoderLocalizer implements Localizer {

    private final TankDrive drive;
    private final HeadingFuser headingFuser;
    private final double trackWidth;
    private final VelocityEstimator leftVelocity = new VelocityEstimator();
    private final VelocityEstimator rightVelocity = new VelocityEstimator();

    private Pose pose = new Pose(0.0, 0.0, 0.0);
    private double lastLeft;
    private double lastRight;
    private double lastHeading;
    private double elapsed;
    private boolean primed;

    public DriveEncoderLocalizer(TankDrive drive, HeadingFuser headingFuser) {
        this.drive = drive;
        this.headingFuser = headingFuser;
        this.trackWidth = drive.getConstants().getTrackWidth();
    }

    @Override
    public void update(double dt) {
        double left = drive.getLeftPositionInches();
        double right = drive.getRightPositionInches();
        elapsed += dt;

        if (!primed) {
            lastLeft = left;
            lastRight = right;
            headingFuser.update(0.0, dt);
            lastHeading = headingFuser.getHeading();
            pose = pose.withHeading(lastHeading);
            leftVelocity.add(elapsed, left);
            rightVelocity.add(elapsed, right);
            primed = true;
            return;
        }

        double deltaLeft = left - lastLeft;
        double deltaRight = right - lastRight;
        lastLeft = left;
        lastRight = right;

        leftVelocity.add(elapsed, left);
        rightVelocity.add(elapsed, right);

        double wheelHeadingDelta = (deltaRight - deltaLeft) / trackWidth;
        headingFuser.update(wheelHeadingDelta, dt);

        double heading = headingFuser.getHeading();
        double deltaHeading = MathUtil.angleDelta(lastHeading, heading);
        lastHeading = heading;

        double forward = (deltaLeft + deltaRight) / 2.0;
        pose = pose.exp(new Twist2(forward, 0.0, deltaHeading));
        // Trust the IMU absolutely rather than the integrated delta, so nothing accumulates.
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
        lastLeft = drive.getLeftPositionInches();
        lastRight = drive.getRightPositionInches();
        leftVelocity.reset();
        rightVelocity.reset();
        primed = true;
    }

    @Override
    public double getForwardVelocity() {
        return (leftVelocity.getVelocity() + rightVelocity.getVelocity()) / 2.0;
    }

    @Override
    public double getLateralVelocity() {
        return 0.0;
    }

    @Override
    public double getAngularVelocity() {
        return headingFuser.getAngularVelocity();
    }

    @Override
    public String status() {
        return "drive encoders + IMU";
    }
}
