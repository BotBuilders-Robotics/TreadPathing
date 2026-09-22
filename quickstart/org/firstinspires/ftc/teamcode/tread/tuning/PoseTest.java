package org.firstinspires.ftc.teamcode.tread.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tread.Constants;
import org.treadpathing.control.PoseHoldController;
import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/**
 * Rung 9: the pose-hold gains, which are what actually set your auto's terminal accuracy.
 *
 * <p>The robot holds a fixed pose. Shove it — sideways, backwards, spin it — and watch it come
 * back. This is the only OpMode in the ladder you tune by hand rather than by reading a log.
 *
 * <ul>
 *   <li><b>Slow to return</b> — raise the distance gain.
 *   <li><b>Overshoots and hunts</b> — lower the bearing gain first, then the distance gain.
 *   <li><b>Spins hard to fix a tiny sideways offset</b> — the blend radius is too large.
 *   <li><b>Ends up in the right place pointing the wrong way</b> — raise the heading gain.
 * </ul>
 *
 * <p>Try pushing it exactly sideways by a couple of inches. That is the hardest thing you can
 * ask a differential drive to fix, because the one direction it cannot move is the one it
 * needs to go, and it has to arc out and back. Expect it to land within about an inch, not on
 * the dot. Chasing that last inch by raising gains buys nothing and costs you the heading.
 */
@TeleOp(name = "Tread 9: Pose Test (hold gains)", group = "tread tuning")
public class PoseTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);
        PoseHoldController hold = follower.getPoseHoldController();

        Pose target = new Pose(72.0, 72.0, 0.0);
        follower.setPose(target);

        telemetry.addLine("The robot will hold its pose. Push it around.");
        telemetry.addLine("A re-anchors the target to where it is now.");
        telemetry.update();
        waitForStart();

        hold.reset();

        while (opModeIsActive()) {
            follower.update();

            if (gamepad1.a) {
                target = follower.getPose();
                hold.reset();
            }

            follower.drive(hold.calculate(follower.getPose(), target, follower.dt()));

            Pose measured = follower.getPose();
            double distance = measured.distanceTo(target);
            double headingError = MathUtil.toDegrees(
                    MathUtil.angleDelta(measured.getHeading(), target.getHeading()));

            telemetry.addData("target", target);
            telemetry.addData("actual", measured);
            telemetry.addLine();
            telemetry.addData("distance (in)", "%.2f", distance);
            telemetry.addData("heading error (deg)", "%.2f", headingError);
            telemetry.addData("at target", hold.atTarget(measured, target) ? "YES" : "no");
            telemetry.addData("loop", "%.0f Hz", follower.getLoopHz());
            telemetry.update();
        }
    }
}
