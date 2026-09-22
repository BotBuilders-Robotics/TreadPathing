package org.treadpathing.holonomic;

/**
 * A mecanum drive's complete command space: two translation terms and a turn rate.
 *
 * <p>The counterpart of {@code ChassisSpeeds}, and the reason a holonomic follower cannot be
 * bolted onto the tank one. Every signature in the control path -- the controller interface,
 * the segment host, the drivetrain -- is typed to a two-number command. A third number is not
 * an extra field; it is a different shape.
 *
 * <p>Both translation terms are in the <b>robot's</b> frame: {@code vx} out of the nose,
 * {@code vy} out of its left side. That is the frame the wheels live in, so it is the frame
 * the kinematics need.
 */
public final class HolonomicSpeeds {

    public static final HolonomicSpeeds ZERO = new HolonomicSpeeds(0.0, 0.0, 0.0);

    private final double vx;
    private final double vy;
    private final double omega;

    /**
     * @param vx    forward speed, inches per second
     * @param vy    left speed, inches per second
     * @param omega turn rate, radians per second, positive counter-clockwise
     */
    public HolonomicSpeeds(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    /** Field-frame velocity turned into a robot-frame command at a given heading. */
    public static HolonomicSpeeds fromField(double fieldVx, double fieldVy, double omega,
                                            double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new HolonomicSpeeds(fieldVx * cos + fieldVy * sin,
                -fieldVx * sin + fieldVy * cos,
                omega);
    }

    public double getVx() {
        return vx;
    }

    public double getVy() {
        return vy;
    }

    public double getOmega() {
        return omega;
    }

    /**
     * The four wheel speeds, in inches per second, front-left, front-right, back-left,
     * back-right.
     *
     * <p>{@code turnRadius} is the sum of the half-track and the half-wheelbase, which is the
     * moment arm a mecanum wheel actually has about the centre of rotation.
     */
    public double[] wheelSpeeds(MecanumKinematics kinematics) {
        double lateral = vy * kinematics.getLateralMultiplier();
        double turn = omega * kinematics.getTurnRadius();
        return new double[] {
                vx - lateral - turn,
                vx + lateral + turn,
                vx + lateral - turn,
                vx - lateral + turn
        };
    }

    /** Largest wheel speed this command asks for, in inches per second. */
    public double peakWheelSpeed(MecanumKinematics kinematics) {
        double[] wheels = wheelSpeeds(kinematics);
        double peak = 0.0;
        for (int i = 0; i < wheels.length; i++) {
            peak = Math.max(peak, Math.abs(wheels[i]));
        }
        return peak;
    }

    /**
     * Scales all three terms together if any wheel would be asked for more than it has.
     *
     * <p>Scaling together rather than clipping the worst wheel is the same call the tank side
     * makes, and matters more here: clipping one wheel of a mecanum does not just change the
     * speed, it changes the <b>direction</b> of travel, so the robot leaves the path sideways.
     */
    public HolonomicSpeeds desaturate(MecanumKinematics kinematics, double maxWheelSpeed) {
        if (maxWheelSpeed <= 0.0) {
            return this;
        }
        double peak = peakWheelSpeed(kinematics);
        if (peak <= maxWheelSpeed) {
            return this;
        }
        double scale = maxWheelSpeed / peak;
        return new HolonomicSpeeds(vx * scale, vy * scale, omega * scale);
    }

    @Override
    public String toString() {
        return String.format("vx=%.2f vy=%.2f in/s, w=%.3f rad/s", vx, vy, omega);
    }
}
