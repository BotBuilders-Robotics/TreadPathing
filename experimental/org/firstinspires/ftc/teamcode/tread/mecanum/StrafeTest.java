package org.firstinspires.ftc.teamcode.tread.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.treadpathing.geometry.Pose;
import org.treadpathing.hardware.BulkReader;
import org.treadpathing.holonomic.MecanumDrive;
import org.treadpathing.localization.Localizer;

/**
 * Measures the lateral multiplier: the rung that has no tank equivalent.
 *
 * <p>The tank ladder's rung 2 is SpinTest, which finds the effective track width because skid
 * steer scrubs in the turn. A mecanum does not scrub in the turn; it scrubs <b>sideways</b>,
 * through the rollers, and this is the number that accounts for it. Everything else on the
 * ladder either transfers unchanged or does not apply.
 *
 * <h3>How it works</h3>
 *
 * Drive forward for a fixed time at a fixed power, and measure how far the robot actually
 * went. Then strafe for the same time at the same power, and measure that. The same wheel
 * speed buys less travel sideways, and the ratio of the two is the multiplier:
 *
 * <pre>
 * lateralMultiplier = forward distance / sideways distance
 * </pre>
 *
 * <p>Both distances come from the <b>localizer</b>, not the drive encoders. Strafing turns the
 * wheels in opposing pairs, so their mean is near zero however far the robot has moved, and
 * the encoders cannot see the thing being measured.
 *
 * <p>For the same reason this needs dead wheels or a Pinpoint, and refuses to run on
 * drive-encoder odometry. {@code MecanumEncoderLocalizer} recovers sideways travel by dividing
 * by the lateral multiplier, so measuring the multiplier through it would just hand back the
 * number already in the constants file, with a plausible run behind it. Switch
 * {@code MecanumConstants.ODOMETRY} to {@code TWO_WHEEL} or {@code PINPOINT} for this test.
 *
 * <h3>Running it</h3>
 *
 * <ol>
 *   <li>Clear about six feet in front of the robot and six feet to its left.
 *   <li>Hold the <b>right bumper</b> for the forward run. Let go; it stops itself.
 *   <li>Hold the <b>left bumper</b> for the strafe. Same power, same duration.
 *   <li>Paste the line it prints into {@link MecanumConstants}.
 * </ol>
 *
 * <p>Repeat it a few times on a fresh battery. A multiplier that moves by more than about 0.05
 * between runs usually means a wheel is barely touching the floor.
 */
@TeleOp(name = "Tread Mecanum 2: Strafe Test (lateral multiplier)", group = "tread experimental")
public class StrafeTest extends LinearOpMode {

    /** Low enough that the wheels do not break traction, which would flatter the result. */
    public static final double POWER = 0.45;

    /** Long enough to swamp the acceleration at either end of the run. */
    public static final double RUN_SECONDS = 2.0;

    @Override
    public void runOpMode() {
        MecanumDrive drive = MecanumConstants.buildDrive(hardwareMap);

        // Refused rather than warned about: a self-referential measurement does not look
        // wrong. It produces the number already in the constants file, with a convincing run
        // behind it, and you would trust it.
        if (MecanumConstants.ODOMETRY == MecanumConstants.Odometry.DRIVE_ENCODERS) {
            telemetry.addLine("This test cannot run on drive-encoder odometry.");
            telemetry.addLine();
            telemetry.addLine("That localizer divides sideways travel BY the lateral");
            telemetry.addLine("multiplier, so measuring the multiplier through it would");
            telemetry.addLine("just return the value already in MecanumConstants.");
            telemetry.addLine();
            telemetry.addLine("Set ODOMETRY to TWO_WHEEL or PINPOINT and run it again.");
            telemetry.update();
            waitForStart();
            return;
        }

        Localizer localizer = MecanumConstants.localizer(hardwareMap, drive);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        double forwardDistance = 0.0;
        double strafeDistance = 0.0;

        telemetry.addLine("Right bumper: drive forward. Left bumper: strafe left.");
        telemetry.addLine("Clear 6 ft ahead and 6 ft to the left.");
        telemetry.update();

        waitForStart();

        double lastTime = time();
        while (opModeIsActive()) {
            bulkReader.clearCache();
            double now = time();
            double dt = clamp(now - lastTime, 0.001, 0.1);
            lastTime = now;
            localizer.update(dt);

            if (gamepad1.right_bumper) {
                forwardDistance = runOnce(drive, localizer, bulkReader, false);
            } else if (gamepad1.left_bumper) {
                strafeDistance = runOnce(drive, localizer, bulkReader, true);
            } else {
                drive.stop();
            }

            telemetry.addData("forward run (in)", "%.2f", forwardDistance);
            telemetry.addData("strafe run (in)", "%.2f", strafeDistance);
            if (forwardDistance > 1.0 && strafeDistance > 1.0) {
                double multiplier = forwardDistance / strafeDistance;
                telemetry.addLine();
                telemetry.addLine("Paste into MecanumConstants:");
                telemetry.addData("  .lateralMultiplier", "%.3f", multiplier);
                if (multiplier < 1.0) {
                    telemetry.addLine("Below 1.0 means the strafe covered MORE ground than the");
                    telemetry.addLine("drive, which no mecanum does. Check the motor order.");
                }
            } else {
                telemetry.addLine();
                telemetry.addLine("Do both runs to get a multiplier.");
            }
            telemetry.update();
        }

        drive.stop();
    }

    /** One timed run at a fixed power. @return distance travelled, in inches */
    private double runOnce(MecanumDrive drive, Localizer localizer, BulkReader bulkReader,
                           boolean strafe) {
        Pose start = localizer.getPose();
        double began = time();
        double lastTime = began;

        while (opModeIsActive() && time() - began < RUN_SECONDS) {
            bulkReader.clearCache();
            double now = time();
            double dt = clamp(now - lastTime, 0.001, 0.1);
            lastTime = now;
            localizer.update(dt);

            if (strafe) {
                drive.setStrafePowers(POWER);
            } else {
                drive.setPowers(POWER);
            }

            telemetry.addLine(strafe ? "strafing..." : "driving...");
            telemetry.addData("elapsed", "%.1f s", time() - began);
            telemetry.update();
        }

        drive.stop();
        Pose end = localizer.getPose();
        // Straight-line distance, so a run that drifts a little is still measured honestly
        // rather than being projected onto an axis the robot did not quite follow.
        return Math.hypot(end.getX() - start.getX(), end.getY() - start.getY());
    }

    private static double clamp(double value, double low, double high) {
        return value < low ? low : (value > high ? high : value);
    }

    private double time() {
        return System.nanoTime() * 1e-9;
    }
}
