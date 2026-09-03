package org.treadpathing.control;

import org.treadpathing.geometry.MathUtil;

/**
 * A small PID with explicit dt, integral clamping and an optional derivative low-pass.
 *
 * <p>Every call takes the measured loop period rather than assuming a fixed one. Java's
 * garbage collector and the FTC SDK make the loop period vary, and integrating a 300 ms
 * hiccup as though it were 20 ms is how a controller ends up doing something violent after a
 * pause.
 */
public final class Pid {

    private final double kP;
    private final double kI;
    private final double kD;
    private final double integralLimit;
    private final double derivativeFilter;

    private double integral;
    private double previousError;
    private double filteredDerivative;
    private boolean hasPrevious;

    public Pid(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, 0.0);
    }

    /**
     * @param integralLimit    magnitude cap on the accumulated integral term's contribution;
     *                         zero disables the integral entirely
     * @param derivativeFilter 0 for a raw derivative, up to just under 1 for heavy smoothing
     */
    public Pid(double kP, double kI, double kD, double integralLimit, double derivativeFilter) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralLimit = integralLimit;
        this.derivativeFilter = MathUtil.clamp(derivativeFilter, 0.0, 0.99);
        reset();
    }

    public void reset() {
        integral = 0.0;
        previousError = 0.0;
        filteredDerivative = 0.0;
        hasPrevious = false;
    }

    public double calculate(double error, double dt) {
        double output = kP * error;

        if (kI != 0.0 && integralLimit > 0.0 && dt > 0.0) {
            integral += error * dt;
            double cap = integralLimit / Math.abs(kI);
            integral = MathUtil.clamp(integral, -cap, cap);
            output += kI * integral;
        }

        if (kD != 0.0 && hasPrevious && dt > 0.0) {
            double raw = (error - previousError) / dt;
            filteredDerivative = derivativeFilter * filteredDerivative + (1.0 - derivativeFilter) * raw;
            output += kD * filteredDerivative;
        }

        previousError = error;
        hasPrevious = true;
        return output;
    }

    public double getKP() {
        return kP;
    }

    public double getKI() {
        return kI;
    }

    public double getKD() {
        return kD;
    }
}
