package org.treadpathing.control;

import org.treadpathing.geometry.MathUtil;

/**
 * The drivetrain's voltage model: {@code V = kS*ramp(v) + kV*v + kA*a}.
 *
 * <p>Because the robot's power output is normalised to [-1, 1] rather than volts, the
 * constants here are in <b>normalised power</b> units:
 *
 * <ul>
 *   <li>{@code kS} — power needed just to overcome stiction, typically 0.05 to 0.15
 *   <li>{@code kV} — power per inch per second, roughly {@code 1 / topSpeed}
 *   <li>{@code kA} — power per inch per second squared, often near 0.002
 * </ul>
 *
 * <p>The static term uses a saturating ramp rather than {@code signum}. The textbook form
 * flips discontinuously between +kS and -kS every time the reference velocity crosses zero,
 * which makes the drivetrain buzz audibly at every stop in a route. Ramping it over a small
 * deadband costs nothing and removes the chatter.
 */
public final class Feedforward {

    /** Velocity below which the static term is ramped rather than applied at full size. */
    public static final double DEFAULT_STATIC_DEADBAND = 1.0;

    private final double kS;
    private final double kV;
    private final double kA;
    private final double staticDeadband;

    public Feedforward(double kS, double kV, double kA) {
        this(kS, kV, kA, DEFAULT_STATIC_DEADBAND);
    }

    /**
     * @param staticDeadband inches per second over which kS ramps in. Zero restores the
     *                       discontinuous textbook behaviour.
     */
    public Feedforward(double kS, double kV, double kA, double staticDeadband) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.staticDeadband = Math.max(staticDeadband, 0.0);
    }

    public double getKS() {
        return kS;
    }

    public double getKV() {
        return kV;
    }

    public double getKA() {
        return kA;
    }

    /**
     * @param velocity     target wheel speed, inches per second
     * @param acceleration target wheel acceleration, inches per second squared
     * @return normalised motor power in [-1, 1] before voltage compensation
     */
    public double calculate(double velocity, double acceleration) {
        return kS * ramp(velocity) + kV * velocity + kA * acceleration;
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0.0);
    }

    /** Saturating replacement for {@code signum}: -1 to +1 across the deadband. */
    public double ramp(double velocity) {
        if (staticDeadband < MathUtil.EPSILON) {
            return velocity == 0.0 ? 0.0 : Math.signum(velocity);
        }
        return MathUtil.clamp(velocity / staticDeadband, -1.0, 1.0);
    }

    /** Inverts the model: the top speed this drivetrain can hold at full power. */
    public double maxAchievableVelocity() {
        if (kV < MathUtil.EPSILON) {
            return 0.0;
        }
        return (1.0 - kS) / kV;
    }
}
