package org.firstinspires.ftc.teamcode.tread.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.hardware.BulkReader;
import org.treadpathing.holonomic.HolonomicController;
import org.treadpathing.holonomic.HolonomicPoseHold;
import org.treadpathing.holonomic.HolonomicRoute;
import org.treadpathing.holonomic.HolonomicSample;
import org.treadpathing.holonomic.HolonomicTrajectory;
import org.treadpathing.holonomic.MecanumDrive;
import org.treadpathing.localization.Localizer;

/**
 * An autonomous for a robot with no odometry hardware at all: four drive encoders and the hub
 * IMU, nothing to buy and nothing to mount.
 *
 * <p>The pair to {@link ExampleMecanumAuto}, which names the Pinpoint and drives a route that
 * earns it. This one names the drive-encoder localizer, and its route is deliberately
 * different, because the odometry underneath it is worse in a specific, predictable way and a
 * route that ignores that will disappoint.
 *
 * <h3>What the route does differently, and why</h3>
 *
 * <b>It travels nose-first.</b> Forward travel is the one thing mecanum wheels measure well:
 * the tread grips and the encoder count means what it says. A strafe is the rollers doing what
 * they are shaped to do, and they slip by an amount that varies with the floor, the wheel wear
 * and the weight on each corner. So the nose follows the path, exactly as a tank robot's would
 * -- not because the drivetrain cannot strafe, but because the <i>odometry</i> cannot measure
 * strafing honestly.
 *
 * <p><b>It turns on the spot rather than while travelling.</b> Heading comes from the IMU and
 * is trustworthy; it is the translation estimate that suffers while the wheels are doing two
 * things at once. Turning where the robot is stationary keeps the two apart.
 *
 * <p><b>It holds at the points that matter.</b> Every pose hold is a chance for the controller
 * to close whatever error the encoders let accumulate on the way there. On this odometry the
 * holds are not polish, they are the error budget.
 *
 * <p>The route is therefore one a tank drive could also follow — which is the honest summary
 * of what a mecanum without pods buys you: the drivetrain can strafe, but you cannot yet
 * <i>trust</i> it to, so save it for teleop and put pods on the robot before the auto depends
 * on it.
 */
@Autonomous(name = "Tread Mecanum Auto: no pods (experimental)", group = "tread experimental")
public class NoPodsAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = MecanumConstants.buildDrive(hardwareMap);
        // Named outright rather than taken from MecanumConstants.ODOMETRY: this OpMode is the
        // no-hardware one, and it should still be that when somebody else has changed the
        // constants to suit their own robot.
        Localizer localizer = MecanumConstants.driveEncoders(hardwareMap, drive);
        BulkReader bulkReader = new BulkReader(hardwareMap);
        HolonomicController controller = MecanumConstants.controller();
        HolonomicPoseHold poseHold = MecanumConstants.poseHold();

        Pose start = new Pose(9.0, 60.0, 0.0);
        localizer.setPose(start);

        HolonomicRoute route = HolonomicRoute.builder(
                        start, MecanumConstants.limits(), drive.getKinematics())
                // Nose along the path the whole way: the direction the encoders measure well.
                .faceTangent()
                .to(34.0, 60.0)
                .to(52.0, 84.0)
                .holdFor(1.0)

                // Turn where the robot is still, then travel nose-first again.
                .turnTo(MathUtil.toRadians(180.0))
                .faceTangent()
                .to(24.0, 84.0)
                .holdFor(1.0)
                .build();

        telemetry.addLine(route.summary());
        telemetry.addLine();
        telemetry.addData("odometry", localizer.status());
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
                            reference.robotAcceleration(pose.getHeading()));
                    if (elapsed >= trajectory.getDuration()) {
                        break;
                    }
                } else {
                    Pose target = leg.isTurn()
                            ? new Pose(leg.getHoldPose().getX(), leg.getHoldPose().getY(),
                                    leg.headingAt(elapsed))
                            : leg.getHoldPose();
                    drive.setSpeeds(poseHold.calculate(pose, target, leg.omegaAt(elapsed)));

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
