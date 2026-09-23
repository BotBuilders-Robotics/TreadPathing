package org.treadpathing.holonomic;

import org.treadpathing.control.Feedforward;

/**
 * A chassis command and its acceleration, turned into four motor powers.
 *
 * <p>Split out of {@link MecanumDrive} so it can be tested without motors. Nothing else in the
 * experiment would notice a kA term applied with the wrong sign: the simulator takes chassis
 * commands, not powers, and a straight line drives perfectly either way.
 */
public final class MecanumFeedforward {

    private final MecanumKinematics kinematics;
    private final Feedforward feedforward;
    private final double maxWheelVelocity;

    public MecanumFeedforward(MecanumDriveConstants constants) {
        this.kinematics = constants.kinematics();
        this.feedforward = new Feedforward(constants.getKS(), constants.getKV(),
                constants.getKA(), constants.getStaticDeadband());
        this.maxWheelVelocity = constants.getMaxWheelVelocity();
    }

    public MecanumKinematics getKinematics() {
        return kinematics;
    }

    /**
     * The fraction of the command the wheels can deliver: 1 unless some wheel would be asked
     * for more than {@code maxWheelVelocity}.
     */
    public double saturationScale(HolonomicSpeeds speeds) {
        double peak = speeds.peakWheelSpeed(kinematics);
        return peak > maxWheelVelocity && maxWheelVelocity > 0.0 ? maxWheelVelocity / peak : 1.0;
    }

    /**
     * Normalised power for each wheel, front-left, front-right, back-left, back-right.
     *
     * <p>Acceleration is mapped through the same kinematics as velocity, so each wheel gets
     * the kA term for <b>its own</b> acceleration, with its own sign. When the command is
     * scaled down to fit the wheels, the acceleration is scaled with it: a robot that cannot
     * reach the speed it was asked for is not accelerating towards it at the planned rate
     * either.
     *
     * @param speeds       robot-frame command
     * @param acceleration robot-frame acceleration, from {@link HolonomicSample#robotAcceleration}
     * @param voltageScale nominal voltage over measured
     */
    public double[] powers(HolonomicSpeeds speeds, HolonomicSpeeds acceleration,
                           double voltageScale) {
        double scale = saturationScale(speeds);
        double[] wheels = speeds.wheelSpeeds(kinematics);
        double[] wheelAccels = acceleration.wheelSpeeds(kinematics);
        double[] out = new double[4];
        double peak = 1.0;
        for (int i = 0; i < 4; i++) {
            out[i] = feedforward.calculate(wheels[i] * scale, wheelAccels[i] * scale) * voltageScale;
            peak = Math.max(peak, Math.abs(out[i]));
        }
        // Desaturation above works in wheel speed, but kS, kA and a sagging battery are added
        // after it and can still push a wheel past full power. Clipping that wheel alone
        // would change the direction of travel, which is the one thing desaturating exists
        // to prevent -- so scale all four together here too.
        for (int i = 0; i < 4; i++) {
            out[i] /= peak;
        }
        return out;
    }

    /** Wheel speeds in inches per second after scaling to fit, for velocity mode. */
    public double[] wheelSpeeds(HolonomicSpeeds speeds) {
        double scale = saturationScale(speeds);
        double[] wheels = speeds.wheelSpeeds(kinematics);
        for (int i = 0; i < 4; i++) {
            wheels[i] *= scale;
        }
        return wheels;
    }
}
