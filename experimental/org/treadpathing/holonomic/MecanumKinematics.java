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
}
