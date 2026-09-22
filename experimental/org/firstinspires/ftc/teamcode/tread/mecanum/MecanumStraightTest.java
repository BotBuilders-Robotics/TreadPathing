package org.firstinspires.ftc.teamcode.tread.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.treadpathing.geometry.Pose;
import org.treadpathing.hardware.BulkReader;
import org.treadpathing.holonomic.MecanumDrive;
import org.treadpathing.localization.Localizer;
import org.treadpathing.util.Datalogger;

/**
 * Measures kA, peak velocity and peak acceleration: the tank ladder's rungs 4 and 5, ported.
 *
 * <p>Steps straight to full power and logs the velocity curve. Three numbers come out:
 *
 * <ul>
 *   <li><b>maxWheelVelocity</b> — the plateau. Put 85% of it in your constants; a trajectory
 *       planned for the absolute peak has no headroom left for the controller to correct with,
 *       and a follower with no headroom cannot follow.
 *   <li><b>maxAcceleration</b> — the steepest part of the rise, near the start.
 *   <li><b>kA</b> — from the time constant of the rise, {@code tau = kA / kV}, so kA is
 *       {@code kV * tau}. The visualizer fits it from the log; the on-screen number is rough.
 * </ul>
 *
 * <h3>The mecanum caveat that has no tank equivalent</h3>
 *
 * These numbers describe the robot going <b>forwards</b>, which is the cheapest direction it
 * has. Strafing costs {@code lateralMultiplier} times as much wheel speed for the same travel,
 * and turning costs more again, so a route that strafes cannot reach the peak measured here.
 * That is not a gap in the measurement: the planner already knows, because
 * {@code HolonomicConstraints} works in wheel speed and derates translation by the angle
 * between travel and the nose. Measure forwards, plan in wheel terms, and the two agree.
 *
 * <p>So run this nose-first, never strafing — a full-power strafe measures the rollers, not
 * the drivetrain.
 */
@TeleOp(name = "Tread Mecanum 4-5: Straight Test (kA, max vel/accel)", group = "tread experimental")
public class MecanumStraightTest extends LinearOpMode {

    public static final double MAX_DISTANCE_INCHES = 100.0;

    /** Set false to run this test without writing a log file. */
    public static final boolean LOGGING = true;

    @Override
    public void runOpMode() {
        MecanumDrive drive = MecanumConstants.buildDrive(hardwareMap);
        Localizer localizer = MecanumConstants.localizer(hardwareMap, drive);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        // The same columns the tank straight test writes, so the visualizer reads it unchanged.
        Datalogger log = LOGGING
                ? new Datalogger("tread_mecanum_straight", new String[] {
                "velocity", "acceleration", "distance", "voltage"})
                : Datalogger.disabled();

        telemetry.addLine("Clear " + (int) MAX_DISTANCE_INCHES + " inches ahead.");
        telemetry.addLine("Hold the right bumper for a full-power run.");
        telemetry.addLine(log.isOpen() ? "Log: " + log.getPath() : "Logging is off.");
        telemetry.update();

        waitForStart();

        double peakVelocity = 0.0;
        double peakAcceleration = 0.0;
        double lastLoop = time();

        try {
            while (opModeIsActive()) {
                bulkReader.clearCache();
                double loopNow = time();
                localizer.update(clamp(loopNow - lastLoop, 0.001, 0.1));
                lastLoop = loopNow;

                if (!gamepad1.right_bumper) {
                    drive.stop();
                    report(peakVelocity, peakAcceleration, drive.getConstants().getKV());
                    telemetry.update();
                    continue;
                }

                double startTime = loopNow;
                Pose start = localizer.getPose();
                double lastVelocity = 0.0;
                double lastTime = startTime;

                while (opModeIsActive() && gamepad1.right_bumper) {
                    bulkReader.clearCache();
                    double now = time();
                    double dt = clamp(now - lastTime, 0.001, 0.1);
                    localizer.update(dt);
                    lastLoop = now;
                    drive.setPowers(1.0);

                    double velocity = localizer.getForwardVelocity();
                    double acceleration = (velocity - lastVelocity) / dt;
                    lastVelocity = velocity;
                    lastTime = now;

                    Pose pose = localizer.getPose();
                    double distance = Math.hypot(pose.getX() - start.getX(),
                            pose.getY() - start.getY());

                    peakVelocity = Math.max(peakVelocity, velocity);
                    // Ignore the first fraction of a second: the derivative of a velocity
                    // estimate that is still filling its window is meaningless.
                    if (now - startTime > 0.15) {
                        peakAcceleration = Math.max(peakAcceleration, acceleration);
                    }

                    log.write(now - startTime, velocity, acceleration, distance,
                            drive.getVoltage());

                    telemetry.addData("velocity (in/s)", "%.1f", velocity);
                    telemetry.addData("distance (in)", "%.1f", distance);
                    telemetry.update();

                    if (distance > MAX_DISTANCE_INCHES) {
                        break;
                    }
                }
                drive.stop();
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
        telemetry.addLine("Paste into MecanumConstants:");
        telemetry.addData("  .maxWheelVelocity", "%.1f", peakVelocity * 0.85);
        telemetry.addData("  .maxVelocity", "%.1f", peakVelocity * 0.85);
        telemetry.addData("  .maxAcceleration", "%.1f", peakAcceleration * 0.85);
        if (peakVelocity > 1.0 && peakAcceleration > 1.0) {
            // v approaches its plateau as 1 - exp(-t/tau) with tau = kA/kV, and the initial
            // slope of that curve is peak/tau, so tau = peak velocity / peak acceleration.
            double tau = peakVelocity / peakAcceleration;
            telemetry.addData("  kA (rough)", "%.5f", kV * tau);
        }
        telemetry.addLine();
        telemetry.addLine("These are forward numbers. Strafing costs the lateral");
        telemetry.addLine("multiplier times as much wheel speed; the planner knows.");
    }

    private static double clamp(double value, double low, double high) {
        return value < low ? low : (value > high ? high : value);
    }

    private double time() {
        return System.nanoTime() * 1e-9;
    }
}
