package org.firstinspires.ftc.teamcode.tread.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tread.Constants;
import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.route.DriveSegment;
import org.treadpathing.route.Route;
import org.treadpathing.route.Segment;
import org.treadpathing.trajectory.TrajectorySample;
import org.treadpathing.util.Datalogger;

/**
 * Rung 7: the follower gains, on a closed 48 inch square.
 *
 * <p>Drives a square and logs planned pose against measured pose every loop. What you are
 * looking for in the log is cross-track error — how far the robot sits to the side of where it
 * was supposed to be:
 *
 * <ul>
 *   <li><b>Corners rounded off, error peaks mid-curve</b> — the follower is too soft. Raise
 *       {@code b} (or the LTV gain scale).
 *   <li><b>Error oscillates on the straights</b> — too hot, or the odometry is noisy. Lower
 *       {@code b}, and check the loop rate before blaming the gains.
 *   <li><b>Error grows steadily and never comes back</b> — this is not a gain problem. Track
 *       width or ticks per inch is wrong; go back to rungs 1 and 2.
 * </ul>
 *
 * <p>The square closes on itself, so the robot should end up where it started. If it does not,
 * the accumulated error is telling you about odometry, not about the controller.
 *
 * <p>Because it returns to its start, this is also the one to leave running while you change
 * one number at a time.
 */
@Autonomous(name = "Tread 7: Square Test (follower gains)", group = "tread tuning")
public class SquareTest extends LinearOpMode {

    public static final double SIDE_INCHES = 48.0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);

        Pose start = new Pose(24.0, 24.0, 0.0);
        follower.setPose(start);

        Route route = follower.route()
                .splineTo(new Pose(24.0 + SIDE_INCHES, 24.0, 0.0))
                .turnToDegrees(90.0)
                .splineTo(new Pose(24.0 + SIDE_INCHES, 24.0 + SIDE_INCHES, MathUtil.toRadians(90.0)))
                .turnToDegrees(180.0)
                .splineTo(new Pose(24.0, 24.0 + SIDE_INCHES, MathUtil.toRadians(180.0)))
                .turnToDegrees(-90.0)
                .splineTo(new Pose(24.0, 24.0, MathUtil.toRadians(-90.0)))
                .turnToDegrees(0.0)
                .stopAndHold(2.0)
                .build();

        telemetry.addLine(route.summary());
        telemetry.addLine();
        telemetry.addLine("Place the robot at (24, 24) facing +x.");
        telemetry.addLine("Log: FIRST/java/src/Datalogs/tread_square.txt");
        telemetry.update();

        waitForStart();

        Datalogger log = new Datalogger("tread_square", new String[] {
                "refX", "refY", "refHeading", "refV",
                "x", "y", "heading", "v",
                "alongTrack", "crossTrack", "headingError", "loopHz"});

        double worstCrossTrack = 0.0;
        double startTime = follower.time();
        follower.follow(route);

        try {
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();

                Segment active = follower.getActiveSegment();
                if (active instanceof DriveSegment) {
                    TrajectorySample reference = ((DriveSegment) active).getLastReference();
                    if (reference != null) {
                        Pose measured = follower.getPose();
                        Pose error = reference.getPose().relativeTo(measured);
                        worstCrossTrack = Math.max(worstCrossTrack, Math.abs(error.getY()));

                        log.write(follower.time() - startTime,
                                reference.getPose().getX(), reference.getPose().getY(),
                                MathUtil.toDegrees(reference.getPose().getHeading()),
                                reference.getVelocity(),
                                measured.getX(), measured.getY(),
                                MathUtil.toDegrees(measured.getHeading()),
                                follower.getForwardVelocity(),
                                error.getX(), error.getY(),
                                MathUtil.toDegrees(error.getHeading()),
                                follower.getLoopHz());

                        telemetry.addData("cross-track (in)", "%.2f", error.getY());
                        telemetry.addData("along-track (in)", "%.2f", error.getX());
                    }
                }
                telemetry.addData("worst cross-track (in)", "%.2f", worstCrossTrack);
                telemetry.addLine(follower.telemetry());
                telemetry.update();
            }
        } finally {
            log.close();
        }

        double closure = follower.getPose().distanceTo(start);
        while (opModeIsActive()) {
            telemetry.addData("worst cross-track (in)", "%.2f", worstCrossTrack);
            telemetry.addData("square closure error (in)", "%.2f", closure);
            telemetry.addLine();
            telemetry.addLine(closure > 3.0
                    ? "Closure is poor - suspect ticksPerInch or trackWidth, not the gains."
                    : "Closure looks fine. Tune gains from the cross-track column.");
            telemetry.update();
        }
    }
}
