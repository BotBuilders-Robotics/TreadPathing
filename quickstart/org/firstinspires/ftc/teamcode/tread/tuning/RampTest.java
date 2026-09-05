package org.firstinspires.ftc.teamcode.tread.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tread.Constants;
import org.treadpathing.follower.Follower;
import org.treadpathing.util.Datalogger;

/**
 * Rung 3: kS and kV, by slow voltage ramp.
 *
 * <p>Ramps power from zero to full over ten seconds while driving straight, logging power
 * against measured wheel speed. Fit a line to the result:
 *
 * <pre>
 *   power = kS + kV * velocity
 * </pre>
 *
 * <p>The intercept is kS, the power needed just to break stiction. The slope is kV, power per
 * inch per second. Drop the log into the visualizer and it does the fit for you, including
 * throwing away the samples below stiction where the robot has not started moving yet.
 *
 * <p>Run it in both directions and average. If the two disagree by more than about ten
 * percent you have a mechanical problem — a dragging bearing, a chain too tight, a wheel out
 * of alignment — and it should be fixed on the robot rather than compensated for in software.
 *
 * <p>Needs a long clear run. It stops itself after {@link #MAX_DISTANCE_INCHES}.
 */
@TeleOp(name = "Tread 3: Ramp Test (kS, kV)", group = "tread tuning")
public class RampTest extends LinearOpMode {

    public static final double RAMP_SECONDS = 10.0;
    public static final double MAX_DISTANCE_INCHES = 110.0;

    /**
     * Set false to run this test without writing a log file. Each run writes a new one --
     * tread_ramp.txt, then tread_ramp_2.txt -- so a directory you never clear out
     * only ever grows.
     */
    public static final boolean LOGGING = true;

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);
        Datalogger log = LOGGING
                ? new Datalogger("tread_ramp", new String[] {
                "power", "leftVel", "rightVel", "avgVel", "distance", "voltage"})
                : Datalogger.disabled();

        telemetry.addLine("Clear " + (int) MAX_DISTANCE_INCHES + " inches ahead of the robot.");
        telemetry.addLine("Right bumper ramps FORWARD, left bumper ramps BACKWARD.");
        telemetry.addLine(log.isOpen() ? "Log: " + log.getPath() : "Logging is off.");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                boolean forward = gamepad1.right_bumper;
                boolean backward = gamepad1.left_bumper;
                if (!forward && !backward) {
                    follower.update();
                    follower.getDrive().stop();
                    telemetry.addLine("Hold a bumper to run the ramp.");
                    telemetry.addData("pose", follower.getPose());
                    telemetry.update();
                    continue;
                }

                double direction = forward ? 1.0 : -1.0;
                double startTime = follower.time();
                double startX = follower.getPose().getX();
                double startY = follower.getPose().getY();

                while (opModeIsActive() && (forward ? gamepad1.right_bumper : gamepad1.left_bumper)) {
                    follower.update();

                    double elapsed = follower.time() - startTime;
                    double power = direction * Math.min(1.0, elapsed / RAMP_SECONDS);
                    follower.getDrive().setPowers(power, power);

                    double dx = follower.getPose().getX() - startX;
                    double dy = follower.getPose().getY() - startY;
                    double distance = Math.sqrt(dx * dx + dy * dy);

                    log.write(elapsed, power,
                            follower.getDrive().getLeftVelocityInches(),
                            follower.getDrive().getRightVelocityInches(),
                            follower.getForwardVelocity(),
                            distance,
                            follower.getDrive().getVoltage());

                    telemetry.addData("power", "%.3f", power);
                    telemetry.addData("velocity (in/s)", "%.1f", follower.getForwardVelocity());
                    telemetry.addData("distance (in)", "%.1f", distance);
                    telemetry.addData("voltage", "%.2f", follower.getDrive().getVoltage());
                    telemetry.update();

                    if (distance > MAX_DISTANCE_INCHES) {
                        break;
                    }
                }
                follower.getDrive().stop();
            }
        } finally {
            // A log that is never closed loses its tail, which is the interesting part.
            log.close();
        }
    }
}
