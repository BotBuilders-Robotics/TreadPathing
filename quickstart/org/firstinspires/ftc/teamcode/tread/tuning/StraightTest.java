package org.firstinspires.ftc.teamcode.tread.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tread.Constants;
import org.treadpathing.follower.Follower;
import org.treadpathing.util.Datalogger;

/**
 * Rungs 4 and 5: kA, peak velocity and peak acceleration, from a full-power step.
 *
 * <p>Steps straight to full power and logs the velocity curve. Three numbers come out:
 *
 * <ul>
 *   <li><b>maxWheelVelocity</b> — the plateau. Put 85% of it in your constants; a trajectory
 *       planned for the absolute peak has no headroom left for the controller to correct with,
 *       and a follower with no headroom cannot follow.
 *   <li><b>maxAcceleration</b> — the steepest part of the rise, near the start.
 *   <li><b>kA</b> — from the time constant of the rise. The velocity approaches its plateau
 *       exponentially with {@code tau = kA / kV}, so kA is {@code kV * tau}. The visualizer
 *       fits this from the log; the on-screen estimate here is rough.
 * </ul>
 *
 * <p>kA is the least important of the three feedforward terms and the hardest to measure. If
 * the fit looks noisy, leave it at 0.002 and move on — the pose controller absorbs what it
 * misses.
 */
@TeleOp(name = "Tread 4-5: Straight Test (kA, max vel/accel)", group = "tread tuning")
public class StraightTest extends LinearOpMode {

    public static final double MAX_DISTANCE_INCHES = 100.0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);
        Datalogger log = new Datalogger("tread_straight", new String[] {
                "velocity", "acceleration", "distance", "voltage"});

        telemetry.addLine("Clear " + (int) MAX_DISTANCE_INCHES + " inches ahead.");
        telemetry.addLine("Hold the right bumper for a full-power run.");
        telemetry.update();

        waitForStart();

        double peakVelocity = 0.0;
        double peakAcceleration = 0.0;

        try {
            while (opModeIsActive()) {
                if (!gamepad1.right_bumper) {
                    follower.update();
                    follower.getDrive().stop();
                    report(peakVelocity, peakAcceleration,
                            follower.getDrive().getConstants().getKV());
                    telemetry.update();
                    continue;
                }

                double startTime = follower.time();
                double startX = follower.getPose().getX();
                double startY = follower.getPose().getY();
                double lastVelocity = 0.0;
                double lastTime = startTime;

                while (opModeIsActive() && gamepad1.right_bumper) {
                    follower.update();
                    follower.getDrive().setPowers(1.0, 1.0);

                    double now = follower.time();
                    double dt = Math.max(now - lastTime, 1e-3);
                    double velocity = follower.getForwardVelocity();
                    double acceleration = (velocity - lastVelocity) / dt;
                    lastVelocity = velocity;
                    lastTime = now;

                    double dx = follower.getPose().getX() - startX;
                    double dy = follower.getPose().getY() - startY;
                    double distance = Math.sqrt(dx * dx + dy * dy);

                    peakVelocity = Math.max(peakVelocity, velocity);
                    // Ignore the first fraction of a second: the derivative of a velocity
                    // estimate that is still filling its window is meaningless.
                    if (now - startTime > 0.15) {
                        peakAcceleration = Math.max(peakAcceleration, acceleration);
                    }

                    log.write(now - startTime, velocity, acceleration, distance,
                            follower.getDrive().getVoltage());

                    telemetry.addData("velocity (in/s)", "%.1f", velocity);
                    telemetry.addData("distance (in)", "%.1f", distance);
                    telemetry.update();

                    if (distance > MAX_DISTANCE_INCHES) {
                        break;
                    }
                }
                follower.getDrive().stop();
            }
        } finally {
            log.close();
        }
    }

    private void report(double peakVelocity, double peakAcceleration, double kV) {
        telemetry.addLine("Hold the right bumper to run.");
        telemetry.addLine();
        telemetry.addData("peak velocity (in/s)", "%.1f", peakVelocity);
        telemetry.addData("peak acceleration (in/s2)", "%.1f", peakAcceleration);
        telemetry.addLine();
        telemetry.addLine("Paste into Constants:");
        telemetry.addData("  .maxWheelVelocity", "%.1f", peakVelocity * 0.85);
        telemetry.addData("  .maxVelocity", "%.1f", peakVelocity * 0.85);
        telemetry.addData("  .maxAcceleration", "%.1f", peakAcceleration * 0.85);
        if (peakVelocity > 1.0 && peakAcceleration > 1.0) {
            // v approaches its plateau as 1 - exp(-t/tau) with tau = kA/kV, and the initial
            // slope of that curve is peak/tau, so tau = peak velocity / peak acceleration.
            double tau = peakVelocity / peakAcceleration;
            telemetry.addData("  kA (rough)", "%.5f", kV * tau);
        }
    }
}
