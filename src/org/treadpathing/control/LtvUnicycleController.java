package org.treadpathing.control;

import org.treadpathing.geometry.Pose;
import org.treadpathing.trajectory.TrajectorySample;

/**
 * The linear time-varying unicycle tracker: feedforward plus an LQR gain scheduled on speed.
 *
 * <p>Structurally identical to {@link RamseteController} — reference velocity plus a gain
 * times the robot-frame error — but the gain is optimal in a least-squares sense rather than
 * merely sufficient for stability, and it is tuned by stating error tolerances instead of
 * picking a number in rad^2/in^2.
 *
 * <p>The gains come from {@link LtvGainTable}, generated offline. Solving the Riccati
 * equation on a Control Hub is not viable inside an OpMode's init.
 *
 * <pre>
 *   v     = v_ref + kx * ex
 *   omega = w_ref + ky * ey + ktheta * etheta
 * </pre>
 */
public final class LtvUnicycleController implements TrajectoryController {

    private final double scale;

    public LtvUnicycleController() {
        this(1.0);
    }

    /**
     * @param gainScale uniform multiplier on every table gain. Start at 1.0; drop toward 0.5
     *                  if the robot buzzes on a straight, which usually means odometry noise
     *                  or loop latency rather than a bad table.
     */
    public LtvUnicycleController(double gainScale) {
        if (gainScale <= 0.0) {
            throw new IllegalArgumentException("gainScale must be positive");
        }
        this.scale = gainScale;
    }

    @Override
    public ChassisSpeeds calculate(Pose measured, TrajectorySample reference) {
        double vRef = reference.getVelocity();
        double wRef = reference.getAngularVelocity();

        Pose error = reference.getPose().relativeTo(measured);
        double[] k = LtvGainTable.gainsFor(vRef);

        double linear = vRef + scale * k[0] * error.getX();
        double angular = wRef
                + scale * k[1] * error.getY()
                + scale * k[2] * error.getHeading();

        return new ChassisSpeeds(linear, angular);
    }

    @Override
    public void reset() {
        // Stateless.
    }

    @Override
    public String name() {
        return "LTV";
    }
}
