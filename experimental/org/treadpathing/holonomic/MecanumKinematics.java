package org.treadpathing.holonomic;

/**
 * The two numbers that turn a chassis command into wheel speeds.
 *
 * <p>Deliberately not {@code DriveConstants}: a tank drive is described by one distance, the
 * effective track width, and a mecanum by two lengths plus a fudge factor for the rollers.
 * Sharing one constants class between them would mean half the fields being meaningless on
 * whichever drivetrain you own.
 */
public final class MecanumKinematics {

    private double turnRadius = 13.0;
    private double lateralMultiplier = 1.15;

    /**
     * @param inches half the track width plus half the wheelbase, which is the moment arm a
     *               mecanum wheel has about the centre of rotation
     */
    public MecanumKinematics turnRadius(double inches) {
        this.turnRadius = inches;
        return this;
    }

    /**
     * Wheel speed needed per inch per second of sideways travel, over and above what forward
     * travel costs.
     *
     * <p>Strafing is lossy: the rollers scrub, and a mecanum robot asked to go sideways at a
     * given speed needs more wheel speed than the ideal kinematics predict. Road Runner calls
     * the same number {@code LATERAL_MULTIPLIER} and tunes it from a strafe test. 1.0 is the
     * textbook value and is always optimistic; measured values sit between 1.1 and 1.5.
     */
    public MecanumKinematics lateralMultiplier(double multiplier) {
        this.lateralMultiplier = multiplier;
        return this;
    }

    public double getTurnRadius() {
        return turnRadius;
    }

    public double getLateralMultiplier() {
        return lateralMultiplier;
    }

    /**
     * Robot-frame motion from four wheel travels: {@code {forward, lateral, rotation}}.
     *
     * <p>The inverse of {@link HolonomicSpeeds#wheelSpeeds}, and kept beside it so the two can
     * be checked against each other. A sign wrong in here does not fail to compile and does
     * not fail on a straight line; it shows up as a robot that drifts while strafing, months
     * later, on a field.
     *
     * <p>Lateral is <b>divided</b> by the lateral multiplier: the wheels turned that far and
     * the robot went less far than they claim. Multiplying would double the error rather than
     * undo it.
     *
     * @param frontLeft  travel of the front-left wheel, inches
     * @param frontRight travel of the front-right wheel, inches
     * @param backLeft   travel of the back-left wheel, inches
     * @param backRight  travel of the back-right wheel, inches
     */
    public double[] forward(double frontLeft, double frontRight,
                            double backLeft, double backRight) {
        double forward = (frontLeft + frontRight + backLeft + backRight) / 4.0;
        double lateral = (-frontLeft + frontRight + backLeft - backRight) / 4.0 / lateralMultiplier;
        double rotation = (-frontLeft + frontRight - backLeft + backRight) / 4.0 / turnRadius;
        return new double[] {forward, lateral, rotation};
    }
}
