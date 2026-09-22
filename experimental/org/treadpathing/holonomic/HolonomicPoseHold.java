package org.treadpathing.holonomic;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/**
 * Drives the last inch, where the trajectory has no authority left because its reference
 * velocity is zero.
 *
 * <p>The tank version of this argument is in the README and it survives the move to mecanum
 * unchanged -- but the primitive is simpler here, because all three axes can be corrected
 * directly instead of a heading correction having to stand in for a lateral one.
 */
public final class HolonomicPoseHold {

    private double translationGain = 3.5;
    private double headingGain = 3.0;
    private double maxSpeed = 24.0;
    private double maxOmega = 2.0;
    private double positionTolerance = 0.5;
    private double headingTolerance = Math.toRadians(1.5);

    public HolonomicPoseHold translationGain(double gain) {
        this.translationGain = gain;
        return this;
    }

    public HolonomicPoseHold headingGain(double gain) {
        this.headingGain = gain;
        return this;
    }

    public HolonomicPoseHold maxSpeed(double inchesPerSecond) {
        this.maxSpeed = inchesPerSecond;
        return this;
    }

    public HolonomicPoseHold maxOmega(double radiansPerSecond) {
        this.maxOmega = radiansPerSecond;
        return this;
    }

    public HolonomicPoseHold tolerance(double inches, double radians) {
        this.positionTolerance = inches;
        this.headingTolerance = radians;
        return this;
    }

    public HolonomicSpeeds calculate(Pose measured, Pose target) {
        double errorX = target.getX() - measured.getX();
        double errorY = target.getY() - measured.getY();
        double errorHeading = MathUtil.angleDelta(measured.getHeading(), target.getHeading());

        double vx = errorX * translationGain;
        double vy = errorY * translationGain;
        double magnitude = Math.hypot(vx, vy);
        if (maxSpeed > 0.0 && magnitude > maxSpeed) {
            double scale = maxSpeed / magnitude;
            vx *= scale;
            vy *= scale;
        }

        double omega = MathUtil.clamp(errorHeading * headingGain, -maxOmega, maxOmega);
        return HolonomicSpeeds.fromField(vx, vy, omega, measured.getHeading());
    }

    public boolean settled(Pose measured, Pose target) {
        double distance = Math.hypot(target.getX() - measured.getX(),
                target.getY() - measured.getY());
        double heading = Math.abs(MathUtil.angleDelta(measured.getHeading(), target.getHeading()));
        return distance <= positionTolerance && heading <= headingTolerance;
    }
}
