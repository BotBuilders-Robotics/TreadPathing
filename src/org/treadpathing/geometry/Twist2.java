package org.treadpathing.geometry;

/**
 * A small robot-frame motion: forward, lateral and rotational displacement measured over
 * one odometry step.
 *
 * <p>{@link #exp()} integrates the twist along a constant-curvature arc rather than a
 * straight line. That difference is what keeps a curving robot's odometry honest — a
 * straight-line integration cuts the inside of every arc and the error compounds.
 */
public final class Twist2 {

    private final double forward;
    private final double lateral;
    private final double rotation;

    public Twist2(double forward, double lateral, double rotation) {
        this.forward = forward;
        this.lateral = lateral;
        this.rotation = rotation;
    }

    public double getForward() {
        return forward;
    }

    public double getLateral() {
        return lateral;
    }

    public double getRotation() {
        return rotation;
    }

    /**
     * Pose exponential: converts this twist into the displacement it produces, expressed in
     * the frame the robot started the step in.
     */
    public Pose exp() {
        double sinTerm;
        double cosTerm;
        if (Math.abs(rotation) < 1e-9) {
            sinTerm = 1.0 - rotation * rotation / 6.0;
            cosTerm = rotation / 2.0;
        } else {
            sinTerm = Math.sin(rotation) / rotation;
            cosTerm = (1.0 - Math.cos(rotation)) / rotation;
        }
        double dx = forward * sinTerm - lateral * cosTerm;
        double dy = forward * cosTerm + lateral * sinTerm;
        return new Pose(dx, dy, rotation);
    }

    @Override
    public String toString() {
        return String.format("Twist(%.4f, %.4f, %.4f)", forward, lateral, rotation);
    }
}
