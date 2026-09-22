package org.treadpathing.holonomic;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/**
 * Feedforward plus proportional correction, in the field frame.
 *
 * <p>Simpler than either tank controller, and that is not an accident. Ramsete and LTV exist
 * because a differential drive cannot correct a sideways error directly: it has to steer, so
 * the correction has to be routed through the heading, and the gain that does that safely
 * needs either a Lyapunov argument or an LQR solution. A mecanum robot can just go sideways.
 * The cross-track error is corrected by driving along it.
 *
 * <p>The other half of the tank library's third principle survives, though: reference velocity
 * still goes to zero at the end of a trajectory, so the last inch still wants a pose hold
 * rather than more gain here.
 */
public final class HolonomicController {

    private double translationGain = 4.0;
    private double headingGain = 3.0;
    private double maxCorrectionSpeed = 30.0;

    /** Correction speed per inch of position error, per second. */
    public HolonomicController translationGain(double gain) {
        this.translationGain = gain;
        return this;
    }

    /** Correction turn rate per radian of heading error, per second. */
    public HolonomicController headingGain(double gain) {
        this.headingGain = gain;
        return this;
    }

    /**
     * Ceiling on the correction term alone, before the feedforward is added.
     *
     * <p>A holonomic controller will happily drive a metre-wide error at full speed straight
     * back at the path, which on a real robot looks like a lurch and ends in an overshoot.
     * Capping the correction leaves the feedforward in charge of the shape of the motion.
     */
    public HolonomicController maxCorrectionSpeed(double inchesPerSecond) {
        this.maxCorrectionSpeed = inchesPerSecond;
        return this;
    }

    public HolonomicSpeeds calculate(Pose measured, HolonomicSample reference) {
        double errorX = reference.getPose().getX() - measured.getX();
        double errorY = reference.getPose().getY() - measured.getY();
        double errorHeading = MathUtil.angleDelta(measured.getHeading(),
                reference.getPose().getHeading());

        double correctionX = errorX * translationGain;
        double correctionY = errorY * translationGain;
        if (maxCorrectionSpeed > 0.0) {
            double magnitude = Math.hypot(correctionX, correctionY);
            if (magnitude > maxCorrectionSpeed) {
                double scale = maxCorrectionSpeed / magnitude;
                correctionX *= scale;
                correctionY *= scale;
            }
        }

        double fieldVx = reference.getFieldVx() + correctionX;
        double fieldVy = reference.getFieldVy() + correctionY;
        double omega = reference.getOmega() + errorHeading * headingGain;

        // Rotated by the measured heading, not the reference one: the wheels are attached to
        // the robot as it actually sits, and using the reference here is the classic way to
        // make a holonomic follower spiral when it is behind on heading.
        return HolonomicSpeeds.fromField(fieldVx, fieldVy, omega, measured.getHeading());
    }

    public String name() {
        return String.format("Holonomic P (kT %.1f, kH %.1f)", translationGain, headingGain);
    }
}
