package org.firstinspires.ftc.teamcode.tread.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tread.Constants;
import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;

/**
 * Rung 2: effective track width.
 *
 * <p>Spin the robot in place with the right stick for several full rotations, then read the
 * number off. The IMU says how far it really turned; the wheels say how far they think it
 * turned; the ratio is the correction.
 *
 * <pre>
 *   effectiveTrackWidth = total wheel difference / total IMU rotation
 * </pre>
 *
 * <p>The answer will be <b>wider than the tape measure</b>, typically by 2 to 4 inches. Skid
 * steer scrubs: the wheels slide sideways through every turn, so the robot rotates less than
 * pure rolling would predict, and the number that makes the kinematics work out is bigger than
 * the physical width. This is not an error to be fixed, it is a property of the drivetrain, and
 * pretending otherwise makes every curve in every auto slightly wrong.
 *
 * <p>Do at least five rotations. The IMU's own error is fixed per reading, so more rotation
 * means proportionally less of it in the answer.
 */
@TeleOp(name = "Tread 2: Spin Test (track width)", group = "tread tuning")
public class SpinTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);
        double nominalTrackWidth = follower.getDrive().getConstants().getTrackWidth();

        telemetry.addLine("Spin the robot in place with the right stick.");
        telemetry.addLine("At least 5 full rotations, either direction. B resets.");
        telemetry.update();

        waitForStart();

        double lastLeft = follower.getDrive().getLeftPositionInches();
        double lastRight = follower.getDrive().getRightPositionInches();
        double lastHeading = follower.getPose().getHeading();
        double wheelDifference = 0.0;
        double imuRotation = 0.0;

        while (opModeIsActive()) {
            follower.update();
            follower.getDrive().setPowers(gamepad1.right_stick_x, -gamepad1.right_stick_x);

            double left = follower.getDrive().getLeftPositionInches();
            double right = follower.getDrive().getRightPositionInches();
            double heading = follower.getPose().getHeading();

            wheelDifference += (right - lastRight) - (left - lastLeft);
            // Accumulate the unwrapped rotation so it keeps counting past 180 degrees.
            imuRotation += MathUtil.angleDelta(lastHeading, heading);

            lastLeft = left;
            lastRight = right;
            lastHeading = heading;

            if (gamepad1.b) {
                wheelDifference = 0.0;
                imuRotation = 0.0;
            }

            telemetry.addData("IMU rotations", "%.2f", imuRotation / MathUtil.TAU);
            telemetry.addData("wheel difference (in)", "%.1f", wheelDifference);
            telemetry.addLine();
            if (Math.abs(imuRotation) > MathUtil.TAU) {
                telemetry.addData("  .trackWidth", "%.3f", wheelDifference / imuRotation);
                telemetry.addData("  vs nominal", "%.2f in", nominalTrackWidth);
            } else {
                telemetry.addLine("Keep spinning - need at least one full rotation.");
            }
            telemetry.addLine();
            telemetry.addLine("Expect 2-4 in WIDER than you measured. That is scrub.");
            telemetry.update();
        }
    }
}
