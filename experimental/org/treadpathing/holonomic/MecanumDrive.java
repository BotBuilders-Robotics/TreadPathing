package org.treadpathing.holonomic;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.treadpathing.control.Feedforward;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.hardware.DriveConstants;

/**
 * Four motors and the kinematics between them and a chassis command.
 *
 * <p>The least uncertain part of the whole experiment: same feedforward, same voltage
 * compensation, same write-caching argument as {@code TankDrive}, four motors instead of two
 * grouped sides.
 *
 * <p>Motor order is front-left, front-right, back-left, back-right throughout, matching
 * {@link HolonomicSpeeds#wheelSpeeds} and {@link MecanumDriveConstants#motors}.
 */
public final class MecanumDrive {

    private final DcMotorEx[] motors;
    private final MecanumDriveConstants constants;
    private final MecanumKinematics kinematics;
    private final Feedforward feedforward;
    private final VoltageSensor voltageSensor;

    private final double[] lastCommand = new double[4];
    private boolean everWritten;
    private double cachedVoltage;
    private int voltageCountdown;

    public MecanumDrive(HardwareMap hardwareMap, MecanumDriveConstants constants) {
        constants.validate();
        this.constants = constants;
        this.kinematics = constants.kinematics();
        this.feedforward = new Feedforward(constants.getKS(), constants.getKV(), constants.getKA(),
                constants.getStaticDeadband());

        String[] names = constants.getMotors();
        boolean[] reversed = constants.getReversed();
        this.motors = new DcMotorEx[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            motors[i].setDirection(reversed[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motors[i].setZeroPowerBehavior(constants.isBrakeOnZeroPower()
                    ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);

            if (constants.getVelocityMode() == DriveConstants.VelocityMode.HUB_PIDF) {
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motors[i].setVelocityPIDFCoefficients(
                        constants.getHubVelocityP(),
                        constants.getHubVelocityI(),
                        constants.getHubVelocityD(),
                        constants.hubVelocityF());
            } else {
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        VoltageSensor sensor = null;
        java.util.Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
        if (sensors.hasNext()) {
            sensor = sensors.next();
        }
        this.voltageSensor = sensor;
        this.cachedVoltage = constants.getNominalVoltage();
    }

    public MecanumKinematics getKinematics() {
        return kinematics;
    }

    public MecanumDriveConstants getConstants() {
        return constants;
    }

    public DcMotorEx[] getMotors() {
        return motors;
    }

    /**
     * @param speeds       what the controller asked for, in the robot's frame
     * @param acceleration reference acceleration along the path, for the kA term
     */
    public void setSpeeds(HolonomicSpeeds speeds, double acceleration) {
        HolonomicSpeeds limited = speeds.desaturate(kinematics, constants.getMaxWheelVelocity());
        double[] wheels = limited.wheelSpeeds(kinematics);

        if (constants.getVelocityMode() == DriveConstants.VelocityMode.HUB_PIDF) {
            for (int i = 0; i < 4; i++) {
                applyVelocity(i, wheels[i] * constants.getTicksPerInch());
            }
            everWritten = true;
            return;
        }

        double scale = voltageScale();
        for (int i = 0; i < 4; i++) {
            double power = MathUtil.clamp(
                    feedforward.calculate(wheels[i], acceleration) * scale, -1.0, 1.0);
            if (shouldWrite(i, power)) {
                motors[i].setPower(power);
                lastCommand[i] = power;
            }
        }
        everWritten = true;
    }

    private void applyVelocity(int index, double ticksPerSecond) {
        // The cache threshold is in power units, so scale it into ticks: the same constant
        // should mean the same thing in both modes.
        double threshold = constants.getMotorCacheThreshold() * constants.getMaxWheelVelocity()
                * constants.getTicksPerInch();
        double last = lastCommand[index];
        boolean write = !everWritten
                || Math.abs(ticksPerSecond - last) >= threshold
                || (ticksPerSecond == 0.0 && last != 0.0);
        if (!write) {
            return;
        }
        motors[index].setVelocity(ticksPerSecond);
        lastCommand[index] = ticksPerSecond;
    }

    public void setSpeeds(HolonomicSpeeds speeds) {
        setSpeeds(speeds, 0.0);
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            if (constants.getVelocityMode() == DriveConstants.VelocityMode.HUB_PIDF) {
                motors[i].setVelocity(0.0);
            } else {
                motors[i].setPower(0.0);
            }
            lastCommand[i] = 0.0;
        }
        everWritten = true;
    }

    // ----- tuning surface ---------------------------------------------------------------
    //
    // The low rungs of the tuning ladder do not use the follower at all: they drive the motors
    // raw and read them back. Without these a mecanum robot cannot measure kS, kV, kA or its
    // own top speed, which means the numbers in its constants file stay placeholders.

    /** Raw power to each corner, bypassing the feedforward. For the tuning OpModes only. */
    public void setPowers(double frontLeft, double frontRight,
                          double backLeft, double backRight) {
        double[] powers = {frontLeft, frontRight, backLeft, backRight};
        for (int i = 0; i < 4; i++) {
            double power = MathUtil.clamp(powers[i], -1.0, 1.0);
            motors[i].setPower(power);
            lastCommand[i] = power;
        }
        everWritten = true;
    }

    /** The same power to all four, which drives straight forwards. */
    public void setPowers(double power) {
        setPowers(power, power, power, power);
    }

    /**
     * Powers that strafe left at the given magnitude.
     *
     * <p>Worth a named method rather than four signs at the call site: getting one of them
     * wrong is the classic mecanum mistake, and a strafe test that quietly drove a diagonal
     * would report a lateral multiplier that is not one.
     */
    public void setStrafePowers(double power) {
        setPowers(-power, power, power, -power);
    }

    /** Per-wheel travel, front-left, front-right, back-left, back-right, in inches. */
    public double[] getWheelPositionsInches() {
        double[] out = new double[4];
        for (int i = 0; i < 4; i++) {
            out[i] = motors[i].getCurrentPosition() / constants.getTicksPerInch();
        }
        return out;
    }

    /** Per-wheel speed in inches per second, in the same order. */
    public double[] getWheelVelocitiesInches() {
        double[] out = new double[4];
        for (int i = 0; i < 4; i++) {
            out[i] = motors[i].getVelocity() / constants.getTicksPerInch();
        }
        return out;
    }

    /**
     * Mean forward travel, in inches.
     *
     * <p>All four wheels turn together going forwards, so the average is the robot's travel.
     * It is <b>not</b> the robot's travel while strafing -- there the wheels turn in opposing
     * pairs and the mean is near zero, which is why a strafe has to be measured with odometry
     * rather than with the drive encoders.
     */
    public double getForwardPositionInches() {
        double[] wheels = getWheelPositionsInches();
        return (wheels[0] + wheels[1] + wheels[2] + wheels[3]) / 4.0;
    }

    /** Mean forward speed, in inches per second, with the same caveat as the position. */
    public double getForwardVelocityInches() {
        double[] wheels = getWheelVelocitiesInches();
        return (wheels[0] + wheels[1] + wheels[2] + wheels[3]) / 4.0;
    }

    /** Battery voltage, refreshed every few loops rather than every loop. */
    public double getVoltage() {
        if (voltageSensor == null) {
            return constants.getNominalVoltage();
        }
        if (voltageCountdown <= 0) {
            double reading = voltageSensor.getVoltage();
            if (reading > 6.0) {
                cachedVoltage = reading;
            }
            voltageCountdown = constants.getVoltageRefreshLoops();
        }
        voltageCountdown--;
        return cachedVoltage;
    }

    /** Skips a write that would not move the motor, which costs a USB round trip each. */
    private boolean shouldWrite(int index, double power) {
        if (!everWritten) {
            return true;
        }
        double last = lastCommand[index];
        return Math.abs(power - last) >= constants.getMotorCacheThreshold()
                || (power == 0.0 && last != 0.0);
    }

    private double voltageScale() {
        return constants.getNominalVoltage() / getVoltage();
    }
}
