package org.firstinspires.ftc.teamcode.tread.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tread.Constants;
import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/**
 * Rung 0 of the tuning ladder: is anything plugged in the right way round?
 *
 * <p>Drive the robot by hand and watch the pose. Three things must be true before any other
 * OpMode here means anything:
 *
 * <ul>
 *   <li>Driving <b>forward</b> increases <b>x</b>
 *   <li>Driving <b>left</b> increases <b>y</b>
 *   <li>Turning <b>counter-clockwise</b> increases <b>heading</b>
 * </ul>
 *
 * <p>If forward decreases x, flip the {@code reversed(left, right)} flags. If the pose moves
 * the wrong way only while turning, a pod offset has the wrong sign. If heading runs backwards,
 * the IMU orientation in {@code Constants.drive().imu(...)} does not match how the hub is
 * actually mounted.
 *
 * <p>Also the fastest way to see your real loop rate, which is the number the whole design
 * budget hangs off.
 */
@TeleOp(name = "Tread 0: Localization Test", group = "tread tuning")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);
        follower.setPose(new Pose(72.0, 72.0, 0.0));

        telemetry.addLine("Drive around and check:");
        telemetry.addLine("  forward -> x up, left -> y up, CCW -> heading up");
        telemetry.addLine("A resets the pose to the field centre.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            double forward = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            follower.getDrive().setPowers(forward - turn, forward + turn);

            if (gamepad1.a) {
                follower.setPose(new Pose(72.0, 72.0, 0.0));
            }

            Pose pose = follower.getPose();
            telemetry.addData("x (in)", "%.2f", pose.getX());
            telemetry.addData("y (in)", "%.2f", pose.getY());
            telemetry.addData("heading (deg)", "%.1f", MathUtil.toDegrees(pose.getHeading()));
            telemetry.addLine();
            telemetry.addData("forward vel (in/s)", "%.1f", follower.getForwardVelocity());
            telemetry.addData("turn rate (deg/s)", "%.1f",
                    MathUtil.toDegrees(follower.getAngularVelocity()));
            telemetry.addLine();
            telemetry.addData("loop", "%.0f Hz   worst %.0f ms",
                    follower.getLoopHz(), follower.getWorstLoopPeriod() * 1000.0);
            telemetry.addData("localizer", follower.getLocalizer().status());
            telemetry.addData("hubs", follower.getBulkReader().hubCount());
            telemetry.update();
        }
    }
}
