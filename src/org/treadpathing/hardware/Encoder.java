package org.treadpathing.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * One encoder, read through the motor port it is plugged into, in inches.
 *
 * <p>Dead wheel pods have no motor; they are wired into a spare motor encoder port and read
 * as though they did. Direction is a property of this wrapper rather than of the motor,
 * because reversing a drive motor's direction to make the robot drive straight would
 * otherwise silently flip the odometry too.
 */
public final class Encoder {

    private final DcMotorEx motor;
    private final double ticksPerInch;
    private final double sign;

    public Encoder(HardwareMap hardwareMap, String name, double ticksPerInch, boolean reversed) {
        this(hardwareMap.get(DcMotorEx.class, name), ticksPerInch, reversed);
    }

    public Encoder(DcMotorEx motor, double ticksPerInch, boolean reversed) {
        if (ticksPerInch <= 0.0) {
            throw new IllegalArgumentException("ticksPerInch must be positive");
        }
        this.motor = motor;
        this.ticksPerInch = ticksPerInch;
        this.sign = reversed ? -1.0 : 1.0;
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public int getRawTicks() {
        return motor.getCurrentPosition();
    }

    public double getPositionInches() {
        return sign * motor.getCurrentPosition() / ticksPerInch;
    }

    /**
     * Velocity straight from the hub, in inches per second.
     *
     * <p>Usable on dead wheels with thousands of counts per revolution. On a bare motor
     * encoder through a big reduction it is mostly quantisation noise at low speed — prefer
     * {@link org.treadpathing.localization.VelocityEstimator} there.
     */
    public double getVelocityInches() {
        return sign * motor.getVelocity() / ticksPerInch;
    }

    public double getTicksPerInch() {
        return ticksPerInch;
    }
}
