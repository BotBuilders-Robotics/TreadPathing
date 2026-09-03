package org.treadpathing.localization;

/**
 * Velocity from a least-squares fit over a short window of position samples.
 *
 * <p>Never take a raw first difference of an FTC encoder. At 50 Hz through a 20:1 gearbox on a
 * bare 28-count motor encoder, a robot creeping at 4 inches per second produces two or three
 * counts per loop; differentiating that measures quantisation, not motion. A regression over
 * five samples costs nothing and gives a signal a controller can actually use.
 *
 * <p>Fitting rather than filtering matters too: an exponential filter lags, and lag in a
 * velocity estimate becomes phase lag in the loop that consumes it. A linear fit over the
 * window has no lag on constant acceleration.
 */
public final class VelocityEstimator {

    public static final int DEFAULT_WINDOW = 5;

    private final double[] times;
    private final double[] positions;
    private final int window;
    private int count;
    private int next;

    public VelocityEstimator() {
        this(DEFAULT_WINDOW);
    }

    public VelocityEstimator(int window) {
        if (window < 2) {
            throw new IllegalArgumentException("Velocity window must be at least 2 samples");
        }
        this.window = window;
        this.times = new double[window];
        this.positions = new double[window];
    }

    public void reset() {
        count = 0;
        next = 0;
    }

    public void add(double time, double position) {
        times[next] = time;
        positions[next] = position;
        next = (next + 1) % window;
        if (count < window) {
            count++;
        }
    }

    /** Slope of the best-fit line through the window. Zero until there are two samples. */
    public double getVelocity() {
        if (count < 2) {
            return 0.0;
        }
        double sumT = 0.0;
        double sumP = 0.0;
        for (int i = 0; i < count; i++) {
            sumT += times[i];
            sumP += positions[i];
        }
        double meanT = sumT / count;
        double meanP = sumP / count;

        double numerator = 0.0;
        double denominator = 0.0;
        for (int i = 0; i < count; i++) {
            double dt = times[i] - meanT;
            numerator += dt * (positions[i] - meanP);
            denominator += dt * dt;
        }
        if (denominator < 1e-12) {
            return 0.0;
        }
        return numerator / denominator;
    }
}
