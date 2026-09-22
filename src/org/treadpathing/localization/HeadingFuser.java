package org.treadpathing.localization;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.hardware.DriveConstants;

/**
 * Heading from the IMU, with wheel odometry filling the gaps between reads.
 *
 * <p><b>Heading comes from the IMU. Always.</b> On a skid-steer chassis, deriving heading from
 * {@code (vRight - vLeft) / trackWidth} accumulates error on every turn, because both sides
 * scrub through the whole turn. Ten degrees of drift over a thirty-second auto is routine.
 * Wheels give translation; the IMU gives theta. This one choice is worth more than any
 * upgrade to the controller.
 *
 * <p>The catch is cost. The IMU is on I2C, which is excluded from bulk reads and takes about
 * 7 ms — a third of the budget at 50 Hz. So it is read every Nth loop and wheel-derived
 * rotation is integrated in between, snapping back to truth whenever a fresh reading lands.
 * The wheel estimate is only ever trusted for a few tens of milliseconds at a time, which is
 * far too short for scrub to matter.
 *
 * <p>Decimation of 1 reads every loop and is always correct; that is the default. Raise it if
 * you need the loop rate and have wheel odometry good enough to coast on.
 */
public final class HeadingFuser {

    private final IMU imu;
    private int decimation;
    private int countdown;

    /**
     * A BNO055 needs roughly 400 to 650 ms after reset before its fusion output is valid, and
     * an OpMode that sets its starting pose in init reads the IMU well inside that window. The
     * SDK does not block or throw: it logs a warning and hands back the identity quaternion,
     * whose yaw is a perfectly innocent-looking zero. Taken as truth that silently rotates the
     * entire run by the robot's real starting heading, and telemetry shows nothing wrong.
     *
     * <p>The tell is {@code getAcquisitionTime()}, which the SDK documents as zero when no
     * reading is associated with the value -- including on the identity fallback.
     */
    private static final long SETTLE_TIMEOUT_MS = 1000L;
    private static final long POLL_INTERVAL_MS = 10L;

    private double offset;
    private double heading;
    private double angularVelocity;
    private double lastImuYaw;
    private boolean primed;

    private boolean settled;
    private boolean offsetPending;
    private double pendingHeading;

    public HeadingFuser(HardwareMap hardwareMap, DriveConstants constants) {
        this(hardwareMap, constants, 1);
    }

    /**
     * @param decimation loops between IMU reads. 1 reads every loop.
     */
    public HeadingFuser(HardwareMap hardwareMap, DriveConstants constants, int decimation) {
        this.imu = hardwareMap.get(IMU.class, constants.getImuName());
        this.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                constants.getLogoFacing(), constants.getUsbFacing())));
        this.decimation = Math.max(1, decimation);
        this.countdown = 0;
    }

    public void setDecimation(int loops) {
        this.decimation = Math.max(1, loops);
    }

    /**
     * @param wheelHeadingDelta rotation since the last call, in radians, as estimated from
     *                          wheel odometry. Pass 0 if the localizer cannot estimate it.
     * @param dt                measured loop period, seconds
     */
    public void update(double wheelHeadingDelta, double dt) {
        if (!primed || countdown <= 0) {
            readImu();
            countdown = decimation;
        } else {
            heading = MathUtil.normalizeAngle(heading + wheelHeadingDelta);
            angularVelocity = dt > 1e-6 ? wheelHeadingDelta / dt : angularVelocity;
        }
        countdown--;
    }

    private void readImu() {
        double yaw = readYaw();
        if (Double.isNaN(yaw)) {
            // Keep the last good heading. Snapping to the offset on a failed read would throw
            // the robot's idea of straight ahead away over one bad I2C transaction.
            return;
        }
        if (offsetPending) {
            offset = MathUtil.normalizeAngle(pendingHeading - yaw);
            offsetPending = false;
        }
        lastImuYaw = yaw;
        heading = MathUtil.normalizeAngle(lastImuYaw + offset);
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        primed = true;
    }

    /**
     * Yaw in radians, or {@code NaN} if the IMU had nothing valid to give.
     *
     * <p>The first call blocks until the IMU comes up, because it is made during init where
     * there is time to spare and a wrong answer costs the whole run. Later calls never block:
     * mid-run a failed read is transient, and the caller keeps the previous heading.
     */
    private double readYaw() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        if (angles.getAcquisitionTime() != 0L) {
            settled = true;
            return angles.getYaw(AngleUnit.RADIANS);
        }
        if (settled) {
            return Double.NaN;
        }

        long deadline = System.nanoTime() + SETTLE_TIMEOUT_MS * 1000000L;
        while (System.nanoTime() < deadline) {
            try {
                Thread.sleep(POLL_INTERVAL_MS);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                return Double.NaN;
            }
            angles = imu.getRobotYawPitchRollAngles();
            if (angles.getAcquisitionTime() != 0L) {
                settled = true;
                return angles.getYaw(AngleUnit.RADIANS);
            }
        }
        return Double.NaN;
    }

    public double getHeading() {
        return heading;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    /**
     * Declares that the robot is currently at this heading. Forces an IMU read so the offset
     * is computed against a fresh reading rather than a stale one.
     */
    public void setHeading(double headingRadians) {
        double yaw = readYaw();
        heading = MathUtil.normalizeAngle(headingRadians);
        if (Double.isNaN(yaw)) {
            // The IMU never came up inside the settle window. Rather than bake in an offset
            // computed against a fabricated zero, remember what was asked for and apply it on
            // the first reading that is real.
            offsetPending = true;
            pendingHeading = headingRadians;
            primed = false;
            countdown = 0;
            return;
        }
        lastImuYaw = yaw;
        offset = MathUtil.normalizeAngle(headingRadians - lastImuYaw);
        offsetPending = false;
        primed = true;
        countdown = decimation;
    }

    /**
     * Zeroes the IMU's own yaw. Only sensible while the robot is completely still — call it
     * during init, never mid-auto.
     */
    public void resetYaw() {
        imu.resetYaw();
        offset = 0.0;
        offsetPending = false;
        primed = false;
        countdown = 0;
    }

    /** Raw IMU yaw with no offset applied, for the tuning OpModes. */
    public double getRawImuYaw() {
        return lastImuYaw;
    }
}
