package org.firstinspires.ftc.teamcode.tread.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tread.Constants;
import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.route.Route;

/**
 * Rung 8: the turn profile and its PID.
 *
 * <p>Turns 90 degrees each way, then 180 each way, and reports where it actually stopped and
 * how much the robot wandered while doing it.
 *
 * <ul>
 *   <li><b>Overshoots</b> — lower {@code turnGains} P, or raise D.
 *   <li><b>Stops short and creeps in</b> — raise P.
 *   <li><b>Oscillates around the target</b> — raise D before touching P.
 *   <li><b>Position drifts during the turn</b> — the track width is off, or one side has more
 *       drag than the other. A turn in place that translates is a mechanical symptom.
 * </ul>
 *
 * <p>A turn in place is a separate primitive precisely because it cannot be a trajectory: a
 * pure rotation covers zero arc length, and the trajectory generator divides by distance
 * travelled.
 */
@Autonomous(name = "Tread 8: Turn Test", group = "tread tuning")
public class TurnTest extends LinearOpMode {

    private static final double[] TURNS_DEGREES = {90.0, -90.0, 180.0, -180.0};

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);
        Pose start = new Pose(72.0, 72.0, 0.0);

        telemetry.addLine("Give the robot clear space. It will turn 90, -90, 180, -180.");
        telemetry.update();
        waitForStart();

        StringBuilder results = new StringBuilder();

        for (int i = 0; i < TURNS_DEGREES.length && opModeIsActive(); i++) {
            follower.setPose(start);

            double target = MathUtil.toRadians(TURNS_DEGREES[i]);
            Route route = follower.routeFrom(start).turn(target).build();

            double began = follower.time();
            double worstDrift = 0.0;
            follower.follow(route);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                worstDrift = Math.max(worstDrift, follower.getPose().distanceTo(start));
                telemetry.addData("turn", "%.0f deg", TURNS_DEGREES[i]);
                telemetry.addLine(follower.telemetry());
                telemetry.update();
            }

            double error = MathUtil.toDegrees(
                    MathUtil.angleDelta(target, follower.getPose().getHeading()));
            results.append(String.format("%6.0f deg -> err %5.2f deg, %.2f s, drift %.2f in%n",
                    TURNS_DEGREES[i], error, follower.time() - began, worstDrift));

            sleep(700);
        }

        while (opModeIsActive()) {
            telemetry.addLine(results.toString());
            telemetry.addLine("Want: error under 2 deg, drift under 0.5 in.");
            telemetry.update();
        }
    }
}
