package org.firstinspires.ftc.teamcode.tread;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.route.Action;
import org.treadpathing.route.InstantAction;
import org.treadpathing.route.Route;

/**
 * A complete autonomous, showing every primitive the library has.
 *
 * <p>Read the route out loud and it describes what the robot does. That is the whole point of
 * the builder.
 *
 * <h3>Three things to notice</h3>
 *
 * <b>Headings are the robot's headings.</b> Every pose is where the nose points, which is what
 * you see standing at the field. On a reversed segment the library flips the travel direction
 * for you.
 *
 * <p><b>{@code reversed()} inserts a cusp.</b> The path being accumulated is closed off, the
 * robot comes to a stop, and a new path starts the other way. That is the only honest way for
 * a differential drive to change direction, and doing it explicitly beats meeting it as a
 * mystery lurch halfway through a match.
 *
 * <p><b>{@code stopAndHold} is what makes it accurate.</b> Drive segments get the robot
 * roughly where it is going, quickly. The pose hold is what gets it exactly there, and it is
 * where you should spend your route's time budget when position matters.
 */
@Autonomous(name = "Tread Example Auto", group = "tread")
public class ExampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);

        Pose start = new Pose(9.0, 60.0, 0.0);
        follower.setPose(start);

        // Anonymous classes rather than lambdas: OnBotJava's Java 8 support is an opt-in
        // checkbox that is off by default, and one stray lambda is a compile error a rookie
        // cannot diagnose at a competition.
        Action raiseArm = new InstantAction() {
            @Override
            public void execute() {
                // arm.setTargetPosition(HIGH);
            }
        };

        Action scoreSample = new Action() {
            private double finishAt;

            @Override
            public void start() {
                // claw.setPosition(OPEN);
                finishAt = System.nanoTime() * 1e-9 + 0.8;
            }

            @Override
            public boolean run() {
                return System.nanoTime() * 1e-9 >= finishAt;
            }
        };

        Route route = follower.route()
                // Out to the scoring position, raising the arm on the way.
                .splineTo(new Pose(34.0, 60.0, 0.0))
                .splineTo(new Pose(52.0, 84.0, MathUtil.toRadians(70.0)))
                .marker(0.60, raiseArm)
                .stopAndHold(1.5)
                .action(scoreSample)

                // Line up to back away, then reverse to the park.
                .turnToDegrees(20.0)
                .reversed()
                .splineTo(new Pose(20.0, 72.0, MathUtil.toRadians(20.0)))
                .stopAndHold(2.0)
                .build();

        telemetry.addLine(route.summary());
        telemetry.addLine();
        telemetry.addData("start", start);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        follower.follow(route);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine(follower.telemetry());
            telemetry.update();
        }

        follower.breakFollowing();

        while (opModeIsActive()) {
            telemetry.addData("finished at", follower.getPose());
            telemetry.addData("planned", route.getPlannedEnd());
            telemetry.addData("error (in)", "%.2f",
                    follower.getPose().distanceTo(route.getPlannedEnd()));
            telemetry.update();
        }
    }
}
