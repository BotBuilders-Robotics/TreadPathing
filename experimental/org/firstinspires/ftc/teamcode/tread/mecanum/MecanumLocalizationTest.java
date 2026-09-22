package org.firstinspires.ftc.teamcode.tread.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.hardware.BulkReader;
import org.treadpathing.holonomic.HolonomicSpeeds;
import org.treadpathing.holonomic.MecanumDrive;
import org.treadpathing.localization.Localizer;

/**
 * Rung 0 of the ladder: is anything plugged in the right way round?
 *
 * <p>Drive the robot by hand and watch the pose. Four things must be true before any other
 * OpMode here means anything:
 *
 * <ul>
 *   <li>Driving <b>forward</b> increases <b>x</b>
 *   <li>Strafing <b>left</b> increases <b>y</b>
 *   <li>Turning <b>counter-clockwise</b> increases <b>heading</b>
 *   <li>Strafing left actually goes <b>sideways</b>, not diagonally
 * </ul>
 *
 * <p>That fourth one is the mecanum addition, and it is the one worth the most attention. Two
 * motors swapped in {@code MecanumConstants.drive().motors(...)} gives a robot that drives
 * forwards perfectly and strafes along a diagonal, or spins when asked to strafe. No amount of
 * tuning fixes it and no validation can catch it — the only way to find it is to command a
 * strafe and look.
 *
 * <p>Drive with the left stick; turn with the right. If the pose moves the wrong way only
 * while turning, a pod offset has the wrong sign. If heading runs backwards, the IMU
 * orientation in {@code MecanumConstants.drive().imu(...)} does not match how the hub is
 * bolted down.
 *
 * <p>Also the fastest way to see your real loop rate, which is the number the whole design
 * budget hangs off.
 */
@TeleOp(name = "Tread Mecanum 0: Localization Test", group = "tread experimental")
public class MecanumLocalizationTest extends LinearOpMode {

    private static final Pose CENTRE = new Pose(72.0, 72.0, 0.0);

    @Override
    public void runOpMode() {
        MecanumDrive drive = MecanumConstants.buildDrive(hardwareMap);
        Localizer localizer = MecanumConstants.localizer(hardwareMap);
        BulkReader bulkReader = new BulkReader(hardwareMap);
        localizer.setPose(CENTRE);

        telemetry.addLine("Left stick drives and strafes, right stick turns.");
        telemetry.addLine("  forward -> x up, left -> y up, CCW -> heading up");
        telemetry.addLine("  and a strafe must go sideways, not diagonally");
        telemetry.addLine("A resets the pose to the field centre.");
        telemetry.update();

        waitForStart();

        double lastTime = time();
        double worstLoop = 0.0;

        while (opModeIsActive()) {
            bulkReader.clearCache();
            double now = time();
            double period = now - lastTime;
            double dt = clamp(period, 0.001, 0.1);
            lastTime = now;
            worstLoop = Math.max(worstLoop, period);

            localizer.update(dt);

            // Driven in the robot's own frame rather than the field's, which is what you want
            // for this test: the sticks should mean the same thing whichever way the robot is
            // pointing, so that a wrong answer is the robot's fault and not the driver's.
            double vx = -gamepad1.left_stick_y;
            double vy = -gamepad1.left_stick_x;
            double omega = -gamepad1.right_stick_x;
            double scale = drive.getConstants().getMaxWheelVelocity();
            drive.setSpeeds(new HolonomicSpeeds(vx * scale, vy * scale, omega * 2.0));

            if (gamepad1.a) {
                localizer.setPose(CENTRE);
            }

            Pose pose = localizer.getPose();
            telemetry.addData("x (in)", "%.2f", pose.getX());
            telemetry.addData("y (in)", "%.2f", pose.getY());
            telemetry.addData("heading (deg)", "%.1f", MathUtil.toDegrees(pose.getHeading()));
            telemetry.addLine();
            telemetry.addData("forward vel (in/s)", "%.1f", localizer.getForwardVelocity());
            // The row a tank drive has no use for: a robot that is strafing is moving sideways
            // in its own frame, and this is where you see whether it really is.
            telemetry.addData("lateral vel (in/s)", "%.1f", localizer.getLateralVelocity());
            telemetry.addData("turn rate (deg/s)", "%.1f",
                    MathUtil.toDegrees(localizer.getAngularVelocity()));
            telemetry.addLine();
            telemetry.addData("loop", "%.0f Hz   worst %.0f ms",
                    period > 1e-6 ? 1.0 / period : 0.0, worstLoop * 1000.0);
            telemetry.addData("localizer", localizer.status());
            telemetry.addData("hubs", bulkReader.hubCount());
            telemetry.update();
        }

        drive.stop();
    }

    private static double clamp(double value, double low, double high) {
        return value < low ? low : (value > high ? high : value);
    }

    private double time() {
        return System.nanoTime() * 1e-9;
    }
}
