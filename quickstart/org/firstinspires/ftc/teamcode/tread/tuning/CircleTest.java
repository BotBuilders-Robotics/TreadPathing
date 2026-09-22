package org.firstinspires.ftc.teamcode.tread.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tread.Constants;
import org.treadpathing.control.ChassisSpeeds;
import org.treadpathing.follower.Follower;

/**
 * Rung 6: maximum centripetal acceleration, which is what stops the robot understeering out
 * of a corner or rolling over.
 *
 * <p>Drives a constant-radius circle at a speed you raise with the right trigger, commanding
 * {@code omega = v / radius}. While the wheels have grip, the measured turn rate matches the
 * commanded one. When traction runs out the robot starts sliding wide and the measured rate
 * falls behind — that is the limit, and the number on screen at that moment is
 * {@code v^2 / radius}.
 *
 * <p>Take about 80% of it. The trajectory generator uses it as
 * {@code v <= sqrt(a_c / |curvature|)}, and a value set right at the slip point means every
 * tight corner in every auto is driven at the edge of control.
 *
 * <p>Run it on the same tiles you compete on. Traction on a clean field and traction on a
 * practice mat that has been walked over all season are not the same number.
 */
@TeleOp(name = "Tread 6: Circle Test (centripetal limit)", group = "tread tuning")
public class CircleTest extends LinearOpMode {

    public static final double RADIUS_INCHES = 24.0;

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);
        double maxSpeed = follower.getDrive().getConstants().getMaxWheelVelocity();

        telemetry.addLine("Clear a circle of radius " + (int) RADIUS_INCHES + " in.");
        telemetry.addLine("Right trigger raises speed. Watch for measured rate falling");
        telemetry.addLine("behind commanded - that is where grip runs out.");
        telemetry.update();

        waitForStart();

        double worstGoodCentripetal = 0.0;

        while (opModeIsActive()) {
            follower.update();

            double speed = gamepad1.right_trigger * maxSpeed * 0.9;
            double commandedRate = speed / RADIUS_INCHES;
            follower.getDrive().setChassisSpeeds(new ChassisSpeeds(speed, commandedRate));

            double measuredRate = follower.getAngularVelocity();
            double centripetal = speed * speed / RADIUS_INCHES;
            boolean gripping = commandedRate < 1e-3
                    || Math.abs(measuredRate) > Math.abs(commandedRate) * 0.9;

            if (gripping && speed > 5.0) {
                worstGoodCentripetal = Math.max(worstGoodCentripetal, centripetal);
            }

            telemetry.addData("speed (in/s)", "%.1f", speed);
            telemetry.addData("commanded rate (rad/s)", "%.3f", commandedRate);
            telemetry.addData("measured rate (rad/s)", "%.3f", measuredRate);
            telemetry.addData("grip", gripping ? "OK" : "SLIPPING");
            telemetry.addLine();
            telemetry.addData("centripetal now (in/s2)", "%.1f", centripetal);
            telemetry.addData("best while gripping", "%.1f", worstGoodCentripetal);
            telemetry.addLine();
            telemetry.addData("  .maxCentripetalAcceleration", "%.1f", worstGoodCentripetal * 0.8);
            telemetry.update();
        }
    }
}
