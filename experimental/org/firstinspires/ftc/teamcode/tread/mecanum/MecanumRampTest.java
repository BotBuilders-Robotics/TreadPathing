package org.firstinspires.ftc.teamcode.tread.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.treadpathing.geometry.Pose;
import org.treadpathing.hardware.BulkReader;
import org.treadpathing.holonomic.MecanumDrive;
import org.treadpathing.localization.Localizer;
import org.treadpathing.util.Datalogger;

/**
 * Measures kS and kV: the tank ladder's rung 3, ported.
 *
 * <p>Ramp the power up slowly, log speed against power, and fit a line. The intercept is kS,
 * the slope is kV. Nothing about that argument is differential-drive specific, so this is the
 * tank RampTest with four wheels instead of two sides — and the log it writes has the same
 * columns, so the visualizer fits it with no changes at all.
 *
 * <p>The ramp is slow on purpose. At this rate the robot is barely accelerating, so the kA
 * term contributes almost nothing and the fit sees kS and kV cleanly. kA comes from a separate
 * full-power run.
 *
 * <ol>
 *   <li>Clear {@code MAX_DISTANCE_INCHES} of floor ahead.
 *   <li>Hold the <b>right bumper</b> to ramp forwards, the <b>left</b> to ramp backwards.
 *   <li>Download the log, drop it on {@code visualizer.html}, and read kS and kV off the fit.
 *   <li>Trust the numbers only if r&sup2; is above about 0.98.
 * </ol>
 */
@TeleOp(name = "Tread Mecanum 3: Ramp Test (kS, kV)", group = "tread experimental")
public class MecanumRampTest extends LinearOpMode {

    public static final double RAMP_SECONDS = 10.0;
    public static final double MAX_DISTANCE_INCHES = 110.0;

    /** Set false to run this test without writing a log file. */
    public static final boolean LOGGING = true;

    @Override
    public void runOpMode() {
        MecanumDrive drive = MecanumConstants.buildDrive(hardwareMap);
        Localizer localizer = MecanumConstants.localizer(hardwareMap);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        // The same columns the tank ramp writes, so the visualizer's fit needs no changes.
        Datalogger log = LOGGING
                ? new Datalogger("tread_mecanum_ramp", new String[] {
                "power", "leftVel", "rightVel", "avgVel", "distance", "voltage"})
                : Datalogger.disabled();

        telemetry.addLine("Clear " + (int) MAX_DISTANCE_INCHES + " inches ahead of the robot.");
        telemetry.addLine("Right bumper ramps FORWARD, left bumper ramps BACKWARD.");
        telemetry.addLine(log.isOpen() ? "Log: " + log.getPath() : "Logging is off.");
        telemetry.update();

        waitForStart();

        double lastTime = time();
        try {
            while (opModeIsActive()) {
                bulkReader.clearCache();
                double now = time();
                double dt = clamp(now - lastTime, 0.001, 0.1);
                lastTime = now;
                localizer.update(dt);

                boolean forward = gamepad1.right_bumper;
                boolean backward = gamepad1.left_bumper;
                if (!forward && !backward) {
                    drive.stop();
                    telemetry.addLine("Hold a bumper to run the ramp.");
                    telemetry.addData("pose", localizer.getPose());
                    telemetry.update();
                    continue;
                }

                double direction = forward ? 1.0 : -1.0;
                double startTime = now;
                Pose start = localizer.getPose();

                while (opModeIsActive() && (forward ? gamepad1.right_bumper : gamepad1.left_bumper)) {
                    bulkReader.clearCache();
                    now = time();
                    dt = clamp(now - lastTime, 0.001, 0.1);
                    lastTime = now;
                    localizer.update(dt);

                    double elapsed = now - startTime;
                    double power = direction * Math.min(1.0, elapsed / RAMP_SECONDS);
                    drive.setPowers(power);

                    Pose pose = localizer.getPose();
                    double distance = Math.hypot(pose.getX() - start.getX(),
                            pose.getY() - start.getY());

                    // Left and right are the means of each side's pair, so the log reads the
                    // same way a tank log does even though there are four wheels behind it.
                    double[] wheels = drive.getWheelVelocitiesInches();
                    double left = (wheels[0] + wheels[2]) / 2.0;
                    double right = (wheels[1] + wheels[3]) / 2.0;

                    log.write(elapsed, power, left, right,
                            localizer.getForwardVelocity(), distance, drive.getVoltage());

                    telemetry.addData("power", "%.3f", power);
                    telemetry.addData("velocity (in/s)", "%.1f", localizer.getForwardVelocity());
                    telemetry.addData("distance (in)", "%.1f", distance);
                    telemetry.addData("voltage", "%.2f", drive.getVoltage());
                    telemetry.update();

                    if (distance > MAX_DISTANCE_INCHES) {
                        break;
                    }
                }
                drive.stop();
            }
        } finally {
            // A log that is never closed loses its tail, which is the interesting part.
            log.close();
        }
    }

    private static double clamp(double value, double low, double high) {
        return value < low ? low : (value > high ? high : value);
    }

    private double time() {
        return System.nanoTime() * 1e-9;
    }
}
