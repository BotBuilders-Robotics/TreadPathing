package org.treadpathing.control;

import org.treadpathing.geometry.MathUtil;

/**
 * Precomputed LQR gains for the LTV unicycle controller, one row per forward velocity.
 *
 * <p>The gains are the solution of a discrete algebraic Riccati equation at each velocity.
 * Solving them on the robot is not an option — WPILib's own constructor runs roughly 1800
 * DARE solves at its defaults, which would blow an OpMode's init budget on a Control Hub.
 * They depend only on the Bryson tolerances, the loop period and velocity, none of which
 * change at runtime, so they are generated offline by {@code tools/gen_ltv_table.py} and
 * pasted here.
 *
 * <p>The gain matrix is structurally sparse: along-track error only drives linear velocity,
 * and cross-track and heading error only drive angular velocity. That is not a
 * simplification, it falls out of the model — so each row stores four numbers rather than
 * eight.
 *
 * <p>Generated with:
 * <pre>
 *   along-track tolerance   8.0 in        linear effort    40.0 in/s
 *   cross-track tolerance  14.0 in        angular effort    1.2 rad/s
 *   heading tolerance       0.6 rad       loop period       0.020 s
 * </pre>
 *
 * <p>Regenerate the table if your loop period is far from 50 Hz or you want a different
 * aggressiveness. Halving a tolerance roughly doubles the corresponding gain.
 *
 * <p><b>Note on standing still:</b> the cross-track gain does not fall to zero as velocity
 * does. At a standstill the controller would happily spin in place trying to fix a lateral
 * error it cannot fix. That is a property of the model, not a bug in the table — it is why
 * routes end in a pose-hold primitive rather than in the trajectory follower.
 */
public final class LtvGainTable {

    /** {velocity in/s, alongTrack gain 1/s, crossTrack gain rad/s/in, heading gain 1/s} */
    private static final double[][] TABLE = {
            {  -72.00,   4.7562461,  -0.0823185,   3.9424882 },
            {  -69.00,   4.7562461,  -0.0823713,   3.8808916 },
            {  -66.00,   4.7562461,  -0.0824250,   3.8182450 },
            {  -63.00,   4.7562461,  -0.0824797,   3.7544951 },
            {  -60.00,   4.7562461,  -0.0825353,   3.6895838 },
            {  -57.00,   4.7562461,  -0.0825920,   3.6234477 },
            {  -54.00,   4.7562461,  -0.0826497,   3.5560176 },
            {  -51.00,   4.7562461,  -0.0827087,   3.4872173 },
            {  -48.00,   4.7562461,  -0.0827689,   3.4169630 },
            {  -45.00,   4.7562461,  -0.0828304,   3.3451620 },
            {  -42.00,   4.7562461,  -0.0828934,   3.2717110 },
            {  -39.00,   4.7562461,  -0.0829578,   3.1964950 },
            {  -36.00,   4.7562461,  -0.0830239,   3.1193850 },
            {  -33.00,   4.7562461,  -0.0830918,   3.0402353 },
            {  -30.00,   4.7562461,  -0.0831615,   2.9588805 },
            {  -27.00,   4.7562461,  -0.0832332,   2.8751317 },
            {  -24.00,   4.7562461,  -0.0833073,   2.7887714 },
            {  -21.00,   4.7562461,  -0.0833837,   2.6995468 },
            {  -18.00,   4.7562461,  -0.0834629,   2.6071618 },
            {  -15.00,   4.7562461,  -0.0835451,   2.5112653 },
            {  -12.00,   4.7562461,  -0.0836306,   2.4114358 },
            {   -9.00,   4.7562461,  -0.0837200,   2.3071605 },
            {   -6.00,   4.7562461,  -0.0838137,   2.1978040 },
            {   -3.00,   4.7562461,  -0.0839125,   2.0825636 },
            {    0.00,   4.7562461,   0.0840171,   1.9604042 },
            {    3.00,   4.7562461,   0.0839125,   2.0825636 },
            {    6.00,   4.7562461,   0.0838137,   2.1978040 },
            {    9.00,   4.7562461,   0.0837200,   2.3071605 },
            {   12.00,   4.7562461,   0.0836306,   2.4114358 },
            {   15.00,   4.7562461,   0.0835451,   2.5112653 },
            {   18.00,   4.7562461,   0.0834629,   2.6071618 },
            {   21.00,   4.7562461,   0.0833837,   2.6995468 },
            {   24.00,   4.7562461,   0.0833073,   2.7887714 },
            {   27.00,   4.7562461,   0.0832332,   2.8751317 },
            {   30.00,   4.7562461,   0.0831615,   2.9588805 },
            {   33.00,   4.7562461,   0.0830918,   3.0402353 },
            {   36.00,   4.7562461,   0.0830239,   3.1193850 },
            {   39.00,   4.7562461,   0.0829578,   3.1964950 },
            {   42.00,   4.7562461,   0.0828934,   3.2717110 },
            {   45.00,   4.7562461,   0.0828304,   3.3451620 },
            {   48.00,   4.7562461,   0.0827689,   3.4169630 },
            {   51.00,   4.7562461,   0.0827087,   3.4872173 },
            {   54.00,   4.7562461,   0.0826497,   3.5560176 },
            {   57.00,   4.7562461,   0.0825920,   3.6234477 },
            {   60.00,   4.7562461,   0.0825353,   3.6895838 },
            {   63.00,   4.7562461,   0.0824797,   3.7544951 },
            {   66.00,   4.7562461,   0.0824250,   3.8182450 },
            {   69.00,   4.7562461,   0.0823713,   3.8808916 },
            {   72.00,   4.7562461,   0.0823185,   3.9424882 },
    };

    private LtvGainTable() {
    }

    public static double minVelocity() {
        return TABLE[0][0];
    }

    public static double maxVelocity() {
        return TABLE[TABLE.length - 1][0];
    }

    /**
     * Linearly interpolates the gain row for a forward velocity, clamping to the ends of the
     * table.
     *
     * @return a three element array: {alongTrack, crossTrack, heading}
     */
    public static double[] gainsFor(double velocity) {
        double v = MathUtil.clamp(velocity, minVelocity(), maxVelocity());

        int low = 0;
        int high = TABLE.length - 1;
        while (high - low > 1) {
            int mid = (low + high) / 2;
            if (TABLE[mid][0] <= v) {
                low = mid;
            } else {
                high = mid;
            }
        }

        double span = TABLE[high][0] - TABLE[low][0];
        double f = span < MathUtil.EPSILON ? 0.0 : (v - TABLE[low][0]) / span;
        return new double[] {
                MathUtil.lerp(TABLE[low][1], TABLE[high][1], f),
                MathUtil.lerp(TABLE[low][2], TABLE[high][2], f),
                MathUtil.lerp(TABLE[low][3], TABLE[high][3], f)
        };
    }
}
