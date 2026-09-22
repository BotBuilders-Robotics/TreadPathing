package org.treadpathing.holonomic;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.treadpathing.hardware.DriveConstants;

/**
 * Everything about a mecanum drivetrain that a team measures once and then forgets.
 *
 * <p>Its own class rather than a few more fields on {@link DriveConstants}, because the two
 * drivetrains disagree about the first thing a constants file says: which motors there are.
 * A differential drive is two <b>sides</b> -- name them and say which one is reversed. A
 * mecanum is four <b>corners</b>, and they are not interchangeable: front-left and back-left
 * turn opposite ways to strafe, so grouping them loses the distinction the drivetrain is
 * built on.
 *
 * <p>The geometry differs the same way. A tank drive is described by one distance, and it is
 * not even a real one -- the effective track width from SpinTest runs two to four inches wider
 * than the wheels physically are, because skid steer scrubs. A mecanum wants two real
 * measurements a tape can take, plus one number for how lossy the rollers are.
 *
 * <pre>
 * MecanumDriveConstants drive = new MecanumDriveConstants()
 *         .motors("leftFront", "rightFront", "leftBack", "rightBack")
 *         .reversed(true, false, true, false)
 *         .trackWidth(14.0)          // tape measure, left wheel centre to right
 *         .wheelBase(12.0)           // tape measure, front wheel centre to back
 *         .lateralMultiplier(1.15)   // from a strafe test, never 1.0 in practice
 *         .feedforward(0.08, 0.0207, 0.0024)
 *         .maxWheelVelocity(36.0);
 * </pre>
 *
 * <p>What does <b>not</b> differ is everything electrical: kS, kV, kA, the deadband, the
 * voltage compensation, the write cache. Those are properties of motors and batteries, not of
 * wheels, and they are duplicated here rather than shared only because this is an experiment
 * living beside the library instead of inside it. A merge would factor them out, and the size
 * of that shared part is part of what the experiment is measuring.
 */
public final class MecanumDriveConstants {

    private String[] motors = {"leftFront", "rightFront", "leftBack", "rightBack"};
    private boolean[] reversed = {true, false, true, false};

    private double trackWidth = 14.0;
    private double wheelBase = 12.0;
    private double lateralMultiplier = 1.15;

    private double ticksPerInch = 30.0;
    private double maxWheelVelocity = 50.0;

    private double kS = 0.08;
    private double kV = 0.018;
    private double kA = 0.002;
    private double staticDeadband = 1.0;

    private DriveConstants.VelocityMode velocityMode = DriveConstants.VelocityMode.FEEDFORWARD;
    private double hubVelocityP = 1.5;
    private double hubVelocityI = 0.1;
    private double hubVelocityD = 0.0;

    private boolean brakeOnZeroPower = true;
    private double motorCacheThreshold = 0.005;
    private double nominalVoltage = 12.0;
    private int voltageRefreshLoops = 10;

    private RevHubOrientationOnRobot.LogoFacingDirection logoFacing =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private RevHubOrientationOnRobot.UsbFacingDirection usbFacing =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private String imuName = "imu";

    // ----- the four corners -------------------------------------------------------------

    /**
     * Configuration names, front-left, front-right, back-left, back-right.
     *
     * <p>Order is load-bearing and there is no way to check it from software: a mecanum with
     * two motors swapped drives perfectly straight and strafes the wrong way, which is the
     * kind of bug that eats an afternoon. Drive each wheel on its own before trusting this.
     */
    public MecanumDriveConstants motors(String frontLeft, String frontRight,
                                        String backLeft, String backRight) {
        this.motors = new String[] {frontLeft, frontRight, backLeft, backRight};
        return this;
    }

    /** Which corners need reversing so that positive power drives that wheel forwards. */
    public MecanumDriveConstants reversed(boolean frontLeft, boolean frontRight,
                                          boolean backLeft, boolean backRight) {
        this.reversed = new boolean[] {frontLeft, frontRight, backLeft, backRight};
        return this;
    }

    // ----- geometry ---------------------------------------------------------------------

    /**
     * Distance between the left and right wheel centres, in inches.
     *
     * <p>The tape measure value, unlike the tank library's, which wants the effective width
     * that skid steer scrub makes wider. A mecanum does not scrub in the turn; it scrubs
     * sideways, and {@link #lateralMultiplier} is where that is accounted for.
     */
    public MecanumDriveConstants trackWidth(double inches) {
        this.trackWidth = inches;
        return this;
    }

    /** Distance between the front and back wheel centres, in inches. */
    public MecanumDriveConstants wheelBase(double inches) {
        this.wheelBase = inches;
        return this;
    }

    /**
     * Wheel speed needed per inch per second of sideways travel, over and above what forward
     * travel costs.
     *
     * <p>Measure it: drive a known distance forwards, then strafe the same distance with the
     * same commanded speed, and divide the time the strafe took by the time the drive took.
     * 1.0 is the textbook value and is always optimistic; real robots land between 1.1 and
     * 1.5 depending on roller wear and how much weight is on each wheel.
     */
    public MecanumDriveConstants lateralMultiplier(double multiplier) {
        this.lateralMultiplier = multiplier;
        return this;
    }

    /**
     * The kinematics these measurements describe.
     *
     * <p>{@code turnRadius} is half the track plus half the wheelbase, which is the moment arm
     * a mecanum wheel has about the centre of rotation. Derived rather than asked for, because
     * a team can measure the two lengths and should not have to average them by hand.
     */
    public MecanumKinematics kinematics() {
        return new MecanumKinematics()
                .turnRadius((trackWidth + wheelBase) / 2.0)
                .lateralMultiplier(lateralMultiplier);
    }

    // ----- the parts that are the same on any drivetrain ---------------------------------

    /** Encoder ticks per inch of wheel travel. Only read in {@code HUB_PIDF} mode. */
    public MecanumDriveConstants ticksPerInch(double value) {
        this.ticksPerInch = value;
        return this;
    }

    /** Peak speed one wheel can hold. Use about 85% of what you measured. */
    public MecanumDriveConstants maxWheelVelocity(double inchesPerSecond) {
        this.maxWheelVelocity = inchesPerSecond;
        return this;
    }

    public MecanumDriveConstants feedforward(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    /** Speed below which kS is not applied, so a stopped robot does not buzz. */
    public MecanumDriveConstants staticDeadband(double inchesPerSecond) {
        this.staticDeadband = inchesPerSecond;
        return this;
    }

    public MecanumDriveConstants velocityMode(DriveConstants.VelocityMode mode) {
        this.velocityMode = mode;
        return this;
    }

    public MecanumDriveConstants hubVelocityPid(double p, double i, double d) {
        this.hubVelocityP = p;
        this.hubVelocityI = i;
        this.hubVelocityD = d;
        return this;
    }

    public MecanumDriveConstants brakeOnZeroPower(boolean brake) {
        this.brakeOnZeroPower = brake;
        return this;
    }

    /** Smallest change in commanded power worth a USB round trip. */
    public MecanumDriveConstants motorCacheThreshold(double threshold) {
        this.motorCacheThreshold = threshold;
        return this;
    }

    public MecanumDriveConstants nominalVoltage(double volts) {
        this.nominalVoltage = volts;
        return this;
    }

    public MecanumDriveConstants voltageRefreshLoops(int loops) {
        this.voltageRefreshLoops = loops;
        return this;
    }

    public MecanumDriveConstants imu(String name,
                                     RevHubOrientationOnRobot.LogoFacingDirection logo,
                                     RevHubOrientationOnRobot.UsbFacingDirection usb) {
        this.imuName = name;
        this.logoFacing = logo;
        this.usbFacing = usb;
        return this;
    }

    // ----- readers ----------------------------------------------------------------------

    public String[] getMotors() {
        return motors;
    }

    public boolean[] getReversed() {
        return reversed;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public double getWheelBase() {
        return wheelBase;
    }

    public double getLateralMultiplier() {
        return lateralMultiplier;
    }

    public double getTicksPerInch() {
        return ticksPerInch;
    }

    public double getMaxWheelVelocity() {
        return maxWheelVelocity;
    }

    public double getKS() {
        return kS;
    }

    public double getKV() {
        return kV;
    }

    public double getKA() {
        return kA;
    }

    public double getStaticDeadband() {
        return staticDeadband;
    }

    public DriveConstants.VelocityMode getVelocityMode() {
        return velocityMode;
    }

    public double getHubVelocityP() {
        return hubVelocityP;
    }

    public double getHubVelocityI() {
        return hubVelocityI;
    }

    public double getHubVelocityD() {
        return hubVelocityD;
    }

    /** The F coefficient in tick units, which is what the hub's PIDF actually wants. */
    public double hubVelocityF() {
        return kV * 32767.0 / ticksPerInch;
    }

    public boolean isBrakeOnZeroPower() {
        return brakeOnZeroPower;
    }

    public double getMotorCacheThreshold() {
        return motorCacheThreshold;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public int getVoltageRefreshLoops() {
        return voltageRefreshLoops;
    }

    public String getImuName() {
        return imuName;
    }

    public RevHubOrientationOnRobot.LogoFacingDirection getLogoFacing() {
        return logoFacing;
    }

    public RevHubOrientationOnRobot.UsbFacingDirection getUsbFacing() {
        return usbFacing;
    }

    /** Fails loudly in init rather than quietly on the field. */
    public void validate() {
        if (motors == null || motors.length != 4) {
            throw new IllegalStateException("A mecanum drive has exactly four motors");
        }
        for (int i = 0; i < motors.length; i++) {
            if (motors[i] == null || motors[i].length() == 0) {
                throw new IllegalStateException("Motor " + i + " has no configuration name");
            }
        }
        if (reversed == null || reversed.length != 4) {
            throw new IllegalStateException("reversed() takes one flag per corner");
        }
        if (trackWidth <= 0.0 || wheelBase <= 0.0) {
            throw new IllegalStateException("trackWidth and wheelBase must be positive");
        }
        if (lateralMultiplier < 1.0) {
            // Below 1.0 says strafing is cheaper than driving, which no mecanum manages.
            throw new IllegalStateException("lateralMultiplier must be at least 1.0");
        }
        if (maxWheelVelocity <= 0.0) {
            throw new IllegalStateException("maxWheelVelocity must be positive");
        }
        if (velocityMode == DriveConstants.VelocityMode.HUB_PIDF && ticksPerInch <= 0.0) {
            throw new IllegalStateException("HUB_PIDF needs ticksPerInch to convert speeds");
        }
    }
}
