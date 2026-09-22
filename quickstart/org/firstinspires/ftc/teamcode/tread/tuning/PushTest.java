package org.firstinspires.ftc.teamcode.tread.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tread.Constants;
import org.treadpathing.follower.Follower;

/**
 * Rung 1: ticks per inch.
 *
 * <p>Motors are left floating so the robot pushes easily. Line it up against a field wall,
 * push it exactly 96 inches (four foam tiles) in a straight line, and read the number off.
 *
 * <p>Measure this; do not compute it from the motor's spec sheet and the wheel diameter. Wheel
 * diameter is nominal, tread compresses under load, and gearboxes are not always what the
 * label says. The measured number is usually a few percent off the calculated one, and that
 * few percent is a couple of inches by the end of an auto.
 */
@TeleOp(name = "Tread 1: Push Test (ticks per inch)", group = "tread tuning")
public class PushTest extends LinearOpMode {

    /** Four foam tiles. Longer is more accurate; use the longest straight run you have. */
    public static final double DISTANCE_INCHES = 96.0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);
        double currentTicksPerInch = follower.getDrive().getConstants().getTicksPerInch();

        telemetry.addLine("Motors are floating. Line the robot up, press start,");
        telemetry.addLine("then push it exactly " + (int) DISTANCE_INCHES + " inches forward.");
        telemetry.update();

        waitForStart();

        double startLeft = follower.getDrive().getLeftPositionInches() * currentTicksPerInch;
        double startRight = follower.getDrive().getRightPositionInches() * currentTicksPerInch;

        while (opModeIsActive()) {
            follower.getBulkReader().clearCache();

            double leftTicks = follower.getDrive().getLeftPositionInches() * currentTicksPerInch - startLeft;
            double rightTicks = follower.getDrive().getRightPositionInches() * currentTicksPerInch - startRight;
            double averageTicks = (leftTicks + rightTicks) / 2.0;

            telemetry.addData("left ticks", "%.0f", leftTicks);
            telemetry.addData("right ticks", "%.0f", rightTicks);
            telemetry.addLine();
            telemetry.addData("reported distance (in)", "%.2f", averageTicks / currentTicksPerInch);
            telemetry.addLine();
            telemetry.addLine("When you have pushed exactly "
                    + (int) DISTANCE_INCHES + " in, paste this:");
            telemetry.addData("  .ticksPerInch", "%.2f", averageTicks / DISTANCE_INCHES);
            telemetry.addLine();
            // A big left/right disagreement is mechanical, not a tuning problem.
            double disagreement = Math.abs(leftTicks - rightTicks)
                    / Math.max(1.0, Math.abs(averageTicks));
            if (disagreement > 0.03) {
                telemetry.addData("WARNING", "sides disagree by %.0f%% - check for drag",
                        disagreement * 100.0);
            }
            telemetry.update();
        }
    }
}
