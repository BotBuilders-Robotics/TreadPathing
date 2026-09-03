package org.treadpathing.control;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.trajectory.TrajectorySample;

/**
 * The Ramsete nonlinear time-varying tracker. This is the library's default follower.
 *
 * <p>With the reference pose expressed in the robot's frame as {@code (ex, ey, etheta)}:
 *
 * <pre>
 *   k     = 2 * zeta * sqrt(w_ref^2 + b * v_ref^2)
 *   v     = v_ref * cos(etheta) + k * ex
 *   omega = w_ref + k * etheta + b * v_ref * sinc(etheta) * ey
 * </pre>
 *
 * <p>The third term is the interesting one. Cross-track error {@code ey} enters the
 * <b>angular</b> command, scaled by reference speed, because turning while moving is the only
 * way a differential drive can close a lateral offset. The controller is built so that
 * {@code V = (b/2)(ex^2 + ey^2) + etheta^2/2} is a Lyapunov function, giving
 * {@code V' = -k*b*ex^2 - k*etheta^2 <= 0}.
 *
 * <p>Ramsete is deprecated in WPILib in favour of LTV, which has more intuitive tuning and
 * optimal rather than merely sufficient gains. It ships as the default here anyway for one
 * practical reason: its gain is a closed form costing a single square root per loop, where
 * LTV needs a precomputed table. Both are available; see {@link LtvUnicycleController}.
 *
 * <p><b>Units.</b> {@code b} carries units of rad^2 per distance^2. WPILib's well-tested
 * default of 2.0 assumes metres; in inches the equivalent is {@code 2.0 / 39.37^2 = 0.00129}.
 * Getting this wrong by a factor of 1550 is the most common Ramsete bug in FTC, so the
 * constructor argument is named for its units and {@link #fromMetricB(double, double)} exists
 * for anyone working from the WPILib literature.
 */
public final class RamseteController implements TrajectoryController {

    /**
     * Default aggressiveness, in rad^2/in^2.
     *
     * <p>WPILib's well-tested value is 2.0 rad^2/m^2, which converts to 0.00129 rad^2/in^2.
     * That value is tuned for FRC robots running at 3 to 4 m/s. Cross-track authority in this
     * control law is {@code b * v}, so a robot that tops out around 1 m/s gets roughly a third
     * of the correction from the same b — which shows up as a tank auto that takes most of a
     * straight to pull back onto its path after a bumped start.
     *
     * <p>This default is scaled up to restore comparable authority at FTC speeds, and lands
     * close to what the LQR solution in {@link LtvGainTable} independently asks for. Teams
     * running unusually fast or unusually noisy odometry should tune it; that is what the
     * SquareTest OpMode is for.
     */
    public static final double DEFAULT_B_PER_INCH_SQUARED = 0.0025;

    public static final double DEFAULT_ZETA = 0.7;

    private static final double INCHES_PER_METER = 39.3700787401575;

    private final double b;
    private final double zeta;

    public RamseteController() {
        this(DEFAULT_B_PER_INCH_SQUARED, DEFAULT_ZETA);
    }

    /**
     * @param bPerInchSquared aggressiveness, in rad^2/in^2. Larger converges harder.
     * @param zeta            damping, dimensionless, strictly between 0 and 1.
     */
    public RamseteController(double bPerInchSquared, double zeta) {
        if (bPerInchSquared <= 0.0) {
            throw new IllegalArgumentException("b must be positive");
        }
        if (zeta <= 0.0 || zeta >= 1.0) {
            throw new IllegalArgumentException("zeta must be strictly between 0 and 1");
        }
        this.b = bPerInchSquared;
        this.zeta = zeta;
    }

    /** Builds a controller from a b quoted in rad^2/m^2, as WPILib and the papers do. */
    public static RamseteController fromMetricB(double bPerMeterSquared, double zeta) {
        return new RamseteController(bPerMeterSquared / (INCHES_PER_METER * INCHES_PER_METER), zeta);
    }

    public double getB() {
        return b;
    }

    public double getZeta() {
        return zeta;
    }

    /** The same b expressed in rad^2/m^2, for comparing against published values. */
    public double getMetricB() {
        return b * INCHES_PER_METER * INCHES_PER_METER;
    }

    @Override
    public ChassisSpeeds calculate(Pose measured, TrajectorySample reference) {
        double vRef = reference.getVelocity();
        double wRef = reference.getAngularVelocity();

        Pose error = reference.getPose().relativeTo(measured);
        double ex = error.getX();
        double ey = error.getY();
        double eTheta = error.getHeading();

        double k = 2.0 * zeta * Math.sqrt(wRef * wRef + b * vRef * vRef);

        double linear = vRef * Math.cos(eTheta) + k * ex;
        double angular = wRef + k * eTheta + b * vRef * MathUtil.sinc(eTheta) * ey;

        return new ChassisSpeeds(linear, angular);
    }

    @Override
    public void reset() {
        // Stateless.
    }

    @Override
    public String name() {
        return "Ramsete";
    }
}
