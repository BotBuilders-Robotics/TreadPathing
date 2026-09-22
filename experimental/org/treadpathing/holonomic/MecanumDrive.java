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
 * grouped sides. Reads {@code DriveConstants} for everything that is genuinely shared -- kS,
 * kV, kA, ticks per inch, nominal voltage -- and takes the geometry from
 * {@link MecanumKinematics}, which is the part that is not.
 *
 * <p>Motor order is front-left, front-right, back-left, back-right throughout, matching
 * {@link HolonomicSpeeds#wheelSpeeds}.
 */
public final class MecanumDrive {

    private final DcMotorEx[] motors;
    private final DriveConstants constants;
    private final MecanumKinematics kinematics;
    private final Feedforward feedforward;
    private final VoltageSensor voltageSensor;

    private final double[] lastCommand = new double[4];
    private boolean everWritten;
    private double cachedVoltage;

    /**
     * @param names front-left, front-right, back-left, back-right, in that order
     */
    public MecanumDrive(HardwareMap hardwareMap, DriveConstants constants,
                        MecanumKinematics kinematics, String[] names, boolean[] reversed) {
        if (names.length != 4 || reversed.length != 4) {
            throw new IllegalArgumentException(
                    "A mecanum drive has exactly four motors, ordered fl, fr, bl, br");
        }
        this.constants = constants;
        this.kinematics = kinematics;
        this.feedforward = new Feedforward(constants.getKS(), constants.getKV(), constants.getKA(),
                constants.getStaticDeadband());

        this.motors = new DcMotorEx[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            motors[i].setDirection(reversed[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motors[i].setZeroPowerBehavior(constants.isBrakeOnZeroPower()
                    ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void setSpeeds(HolonomicSpeeds speeds) {
        setSpeeds(speeds, 0.0);
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(0.0);
            lastCommand[i] = 0.0;
        }
        everWritten = true;
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
        if (voltageSensor != null) {
            double measured = voltageSensor.getVoltage();
            if (measured > 6.0) {
                cachedVoltage = measured;
            }
        }
        return constants.getNominalVoltage() / cachedVoltage;
    }
}
