package org.treadpathing.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.treadpathing.control.ChassisSpeeds;
import org.treadpathing.control.Feedforward;
import org.treadpathing.geometry.MathUtil;

/**
 * The drivetrain: chassis speeds in, motor commands out.
 *
 * <p>Three things happen between the two, and each of them is worth a match:
 *
 * <ul>
 *   <li><b>Desaturation.</b> If a command would ask either side for more than it has, both
 *       terms scale down together. Clipping only the faster wheel would quietly change the arc
 *       the robot drives — it would still move, just not where the trajectory said.
 *   <li><b>Voltage compensation.</b> Four drive motors under acceleration can pull a pack from
 *       12.8 V to 10.5 V. Without scaling by measured voltage, a kV characterised on a fresh
 *       battery is 20% wrong by the last match of the day, and the auto that worked in the
 *       morning misses.
 *   <li><b>Write caching.</b> A motor write costs a hub command. Skipping writes that barely
 *       changed means most loops on a straight write nothing at all.
 * </ul>
 */
public final class TankDrive {

    private final DcMotorEx[] left;
    private final DcMotorEx[] right;
    private final DriveConstants constants;
    private final Feedforward feedforward;
    private final VoltageSensor voltageSensor;

    private double lastLeftCommand = Double.NaN;
    private double lastRightCommand = Double.NaN;
    private double cachedVoltage;
    private int voltageCountdown;

    public TankDrive(HardwareMap hardwareMap, DriveConstants constants) {
        this.constants = constants;
        this.feedforward = new Feedforward(constants.getKS(), constants.getKV(), constants.getKA(),
                constants.getStaticDeadband());

        this.left = fetch(hardwareMap, constants.getLeftMotors());
        this.right = fetch(hardwareMap, constants.getRightMotors());

        configure(left, constants.isLeftReversed());
        configure(right, constants.isRightReversed());

        VoltageSensor sensor = null;
        java.util.Iterator<VoltageSensor> sensors = hardwareMap.voltageSensor.iterator();
        if (sensors.hasNext()) {
            sensor = sensors.next();
        }
        this.voltageSensor = sensor;
        this.cachedVoltage = constants.getNominalVoltage();
        this.voltageCountdown = 0;
    }

    private static DcMotorEx[] fetch(HardwareMap hardwareMap, String[] names) {
        DcMotorEx[] motors = new DcMotorEx[names.length];
        for (int i = 0; i < names.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
        }
        return motors;
    }

    private void configure(DcMotorEx[] motors, boolean reversed) {
        for (int i = 0; i < motors.length; i++) {
            DcMotorEx motor = motors[i];
            motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(constants.isBrakeOnZeroPower()
                    ? DcMotor.ZeroPowerBehavior.BRAKE
                    : DcMotor.ZeroPowerBehavior.FLOAT);

            if (constants.getVelocityMode() == DriveConstants.VelocityMode.HUB_PIDF) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setVelocityPIDFCoefficients(
                        constants.getHubVelocityP(),
                        constants.getHubVelocityI(),
                        constants.getHubVelocityD(),
                        constants.hubVelocityF());
            } else {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public DriveConstants getConstants() {
        return constants;
    }

    // ----- commanding -------------------------------------------------------------------

    /**
     * @param speeds             what the controller asked for
     * @param linearAcceleration reference forward acceleration, for the kA term; pass 0 if
     *                           you do not have it
     */
    public void setChassisSpeeds(ChassisSpeeds speeds, double linearAcceleration) {
        ChassisSpeeds limited = speeds.desaturate(constants.getTrackWidth(), constants.getMaxWheelVelocity());
        setWheelVelocities(
                limited.leftWheelSpeed(constants.getTrackWidth()),
                limited.rightWheelSpeed(constants.getTrackWidth()),
                linearAcceleration);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setChassisSpeeds(speeds, 0.0);
    }

    /** Wheel speeds in inches per second, plus a shared acceleration feedforward. */
    public void setWheelVelocities(double leftInchesPerSecond, double rightInchesPerSecond,
                                   double acceleration) {
        if (constants.getVelocityMode() == DriveConstants.VelocityMode.HUB_PIDF) {
            applyVelocity(left, leftInchesPerSecond * constants.getTicksPerInch(), true);
            applyVelocity(right, rightInchesPerSecond * constants.getTicksPerInch(), false);
            return;
        }

        double scale = voltageScale();
        double leftPower = MathUtil.clamp(
                feedforward.calculate(leftInchesPerSecond, acceleration) * scale, -1.0, 1.0);
        double rightPower = MathUtil.clamp(
                feedforward.calculate(rightInchesPerSecond, acceleration) * scale, -1.0, 1.0);
        applyPower(left, leftPower, true);
        applyPower(right, rightPower, false);
    }

    /** Raw open-loop power, bypassing the feedforward. Used by the tuning OpModes. */
    public void setPowers(double leftPower, double rightPower) {
        applyPower(left, MathUtil.clamp(leftPower, -1.0, 1.0), true);
        applyPower(right, MathUtil.clamp(rightPower, -1.0, 1.0), false);
    }

    public void stop() {
        if (constants.getVelocityMode() == DriveConstants.VelocityMode.HUB_PIDF) {
            applyVelocity(left, 0.0, true);
            applyVelocity(right, 0.0, false);
        } else {
            applyPower(left, 0.0, true);
            applyPower(right, 0.0, false);
        }
    }

    private void applyPower(DcMotorEx[] motors, double power, boolean isLeft) {
        if (!shouldWrite(power, isLeft)) {
            return;
        }
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(power);
        }
        remember(power, isLeft);
    }

    private void applyVelocity(DcMotorEx[] motors, double ticksPerSecond, boolean isLeft) {
        // The cache threshold is in power units; scale it into ticks so the same constant
        // means the same thing in both modes.
        double threshold = constants.getMotorCacheThreshold() * constants.getMaxWheelVelocity()
                * constants.getTicksPerInch();
        double last = isLeft ? lastLeftCommand : lastRightCommand;
        boolean write = Double.isNaN(last)
                || Math.abs(ticksPerSecond - last) >= threshold
                || (ticksPerSecond == 0.0 && last != 0.0);
        if (!write) {
            return;
        }
        for (int i = 0; i < motors.length; i++) {
            motors[i].setVelocity(ticksPerSecond);
        }
        remember(ticksPerSecond, isLeft);
    }

    private boolean shouldWrite(double command, boolean isLeft) {
        double last = isLeft ? lastLeftCommand : lastRightCommand;
        if (Double.isNaN(last)) {
            return true;
        }
        // Always let a stop through: a cached near-zero power that never becomes exactly zero
        // is how a robot creeps at the end of an auto.
        if (command == 0.0 && last != 0.0) {
            return true;
        }
        return Math.abs(command - last) >= constants.getMotorCacheThreshold();
    }

    private void remember(double command, boolean isLeft) {
        if (isLeft) {
            lastLeftCommand = command;
        } else {
            lastRightCommand = command;
        }
    }

    // ----- sensing ----------------------------------------------------------------------

    public double getLeftPositionInches() {
        return averagePosition(left, constants.isLeftEncoderReversed());
    }

    public double getRightPositionInches() {
        return averagePosition(right, constants.isRightEncoderReversed());
    }

    public double getLeftVelocityInches() {
        return averageVelocity(left, constants.isLeftEncoderReversed());
    }

    public double getRightVelocityInches() {
        return averageVelocity(right, constants.isRightEncoderReversed());
    }

    private double averagePosition(DcMotorEx[] motors, boolean encoderReversed) {
        double sum = 0.0;
        for (int i = 0; i < motors.length; i++) {
            sum += motors[i].getCurrentPosition();
        }
        double inches = sum / (motors.length * constants.getTicksPerInch());
        return encoderReversed ? -inches : inches;
    }

    private double averageVelocity(DcMotorEx[] motors, boolean encoderReversed) {
        double sum = 0.0;
        for (int i = 0; i < motors.length; i++) {
            sum += motors[i].getVelocity();
        }
        double inchesPerSecond = sum / (motors.length * constants.getTicksPerInch());
        // Position and velocity must carry the same sign convention, or the follower will
        // integrate one and correct against the other.
        return encoderReversed ? -inchesPerSecond : inchesPerSecond;
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
            voltageCountdown = Math.max(1, constants.getVoltageRefreshLoops());
        }
        voltageCountdown--;
        return cachedVoltage;
    }

    private double voltageScale() {
        double voltage = getVoltage();
        if (voltage < 6.0) {
            return 1.0;
        }
        return MathUtil.clamp(constants.getNominalVoltage() / voltage, 0.5, 2.0);
    }
}
