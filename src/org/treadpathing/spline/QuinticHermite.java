package org.treadpathing.spline;

/**
 * A one-dimensional quintic Hermite polynomial on t in [0, 1], fixed by the value, first
 * derivative and second derivative at each end.
 *
 * <p>Quintic rather than cubic because heading is {@code atan2(y', x')}: taking the heading
 * of a curve costs one order of continuity, so a C2 position spline only gives C1 heading —
 * which means discontinuous angular acceleration, which means a step demand in differential
 * wheel torque that no motor can deliver. Quintics with matched second derivatives buy back
 * that order.
 */
public final class QuinticHermite {

    private final double c0;
    private final double c1;
    private final double c2;
    private final double c3;
    private final double c4;
    private final double c5;

    public QuinticHermite(double p0, double v0, double a0, double p1, double v1, double a1) {
        this.c0 = p0;
        this.c1 = v0;
        this.c2 = a0 / 2.0;
        this.c3 = -10.0 * p0 - 6.0 * v0 - 1.5 * a0 + 10.0 * p1 - 4.0 * v1 + 0.5 * a1;
        this.c4 = 15.0 * p0 + 8.0 * v0 + 1.5 * a0 - 15.0 * p1 + 7.0 * v1 - 1.0 * a1;
        this.c5 = -6.0 * p0 - 3.0 * v0 - 0.5 * a0 + 6.0 * p1 - 3.0 * v1 + 0.5 * a1;
    }

    public double value(double t) {
        return c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * c5))));
    }

    public double derivative(double t) {
        return c1 + t * (2.0 * c2 + t * (3.0 * c3 + t * (4.0 * c4 + t * 5.0 * c5)));
    }

    public double secondDerivative(double t) {
        return 2.0 * c2 + t * (6.0 * c3 + t * (12.0 * c4 + t * 20.0 * c5));
    }

    public double thirdDerivative(double t) {
        return 6.0 * c3 + t * (24.0 * c4 + t * 60.0 * c5);
    }
}
