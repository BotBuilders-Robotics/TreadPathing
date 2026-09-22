package org.firstinspires.ftc.teamcode.tread.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.hardware.BulkReader;
import org.treadpathing.holonomic.HeadingPlans;
import org.treadpathing.holonomic.HolonomicController;
import org.treadpathing.holonomic.HolonomicPoseHold;
import org.treadpathing.holonomic.HolonomicRoute;
import org.treadpathing.holonomic.HolonomicSample;
import org.treadpathing.holonomic.HolonomicTrajectory;
import org.treadpathing.holonomic.MecanumDrive;
import org.treadpathing.localization.Localizer;

/**
 * A complete mecanum autonomous, and an argument for why the experiment is not finished.
 *
 * <p>The route reads the way the tank one does, and says things a tank route cannot: strafe
 * while facing the goal, curve out while the nose swings round. That part works.
 *
 * <p>The loop underneath does not read that way, and that is the point. On the tank side this
 * whole method is {@code follower.update()} inside a {@code while (follower.isBusy())}, because
 * {@code Follower} owns the localizer, the clock, the segment cursor and the drivetrain. There
 * is no holonomic {@code Follower}, so all of it is written out by hand here: reading the
 * localizer, timing the loop, walking the legs, sampling the trajectory, and deciding when a
 * leg is done. Nobody should have to write this to drive a route.
 *
 * <p>Left inline deliberately rather than tidied into a helper class, because a holonomic
 * follower <b>is</b> that helper class, and the reason there is not one is in
 * {@code experimental/README.md}: every signature from the controller interface down to the
 * drivetrain is typed to a two-number chassis command.
 */
@Autonomous(name = "Tread Mecanum Example (experimental)", group = "tread experimental")
public class ExampleMecanumAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = MecanumConstants.buildDrive(hardwareMap);
        Localizer localizer = MecanumConstants.localizer(hardwareMap);
        BulkReader bulkReader = new BulkReader(hardwareMap);
        HolonomicController controller = MecanumConstants.controller();
        HolonomicPoseHold poseHold = MecanumConstants.poseHold();

        Pose start = new Pose(9.0, 60.0, 0.0);
        localizer.setPose(start);

        HolonomicRoute route = HolonomicRoute.builder(
                        start, MecanumConstants.limits(), drive.getKinematics())
                // Out to the scoring position sideways, nose fixed on the goal the whole way.
                // A tank robot cannot express this line at all.
                .faceAngle(MathUtil.toRadians(90.0))
                .to(34.0, 60.0)
                .to(52.0, 84.0)
                .holdFor(1.0)

                // Then away, with the nose coming round to face the next task as it travels,
                // rather than stopping to turn.
                .withPlan(HeadingPlans.interpolate(
                        MathUtil.toRadians(90.0), MathUtil.toRadians(200.0)))
                .to(30.0, 100.0)
                .turnTo(MathUtil.toRadians(180.0))
                .holdFor(0.5)
                .build();

        telemetry.addLine(route.summary());
        telemetry.update();

        waitForStart();

        double lastTime = time();
        for (int index = 0; index < route.size() && opModeIsActive(); index++) {
            HolonomicRoute.Leg leg = route.get(index);
            double legStart = time();

            while (opModeIsActive()) {
                bulkReader.clearCache();

                double now = time();
                double dt = MathUtil.clamp(now - lastTime, 0.001, 0.1);
                lastTime = now;

                localizer.update(dt);
                Pose pose = localizer.getPose();
                double elapsed = now - legStart;

                if (leg.isDrive()) {
                    HolonomicTrajectory trajectory = leg.getTrajectory();
                    HolonomicSample reference = trajectory.sample(elapsed);
                    drive.setSpeeds(controller.calculate(pose, reference),
                            reference.getAcceleration());
                    if (elapsed >= trajectory.getDuration()) {
                        break;
                    }
                } else {
                    // A turn in place walks its own profile; a hold simply sits on the pose.
                    Pose target = leg.isTurn()
                            ? new Pose(leg.getHoldPose().getX(), leg.getHoldPose().getY(),
                                    leg.headingAt(elapsed))
                            : leg.getHoldPose();
                    drive.setSpeeds(poseHold.calculate(pose, target));

                    // Settle on tolerance, not on the clock: the profile finishing is not the
                    // same as the robot arriving, and a turn that ends on time ends short.
                    boolean done = elapsed >= leg.getHoldSeconds()
                            && (!leg.isTurn() || poseHold.settled(pose, target));
                    if (done || elapsed >= leg.getHoldSeconds() + 1.5) {
                        break;
                    }
                }

                telemetry.addData("leg", (index + 1) + "/" + route.size() + "  " + leg.describe());
                telemetry.addData("pose", pose);
                telemetry.update();
            }
        }

        drive.stop();
    }

    private double time() {
        return System.nanoTime() * 1e-9;
    }
}
