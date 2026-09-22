package org.treadpathing.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Everything about the drivetrain that a team measures once and then forgets.
 *
 * <p>Fluent setters so a team's {@code Constants} class reads as a description of the robot.
 * Every distance is inches, every angle radians, every time seconds.
 */
public final class DriveConstants {

    /** How wheel velocity is closed: in Java, or on the hub. */
    public enum VelocityMode {
        /**
         * Open loop. Wheel speed goes through {@code kS/kV/kA} and comes out as motor power.
         * Simplest, no encoder noise in the loop, and the pose controller upstream cleans up
         * what is left. Start here.
         */
        FEEDFORWARD,
        /**
         * {@code RUN_USING_ENCODER} plus {@code setVelocity}, so the hub's own PIDF closes the
         * loop. The hub runs far faster than a 50 Hz OpMode and sees better encoder data, so
         * this usually tracks better — but it needs the F coefficient set from kV in tick
         * units, which {@link TankDrive} does for you.
         */
        HUB_PIDF
    }

    private String[] leftMotors = {"leftFront", "leftBack"};
    private String[] rightMotors = {"rightFront", "rightBack"};
    private boolean leftReversed = true;
    private boolean rightReversed = false;
    private boolean leftEncoderReversed = false;
    private boolean rightEncoderReversed = false;

    private double ticksPerInch = 30.0;
    private double trackWidth = 14.0;
    private double maxWheelVelocity = 50.0;

    private double kS = 0.08;
    private double kV = 0.018;
    private double kA = 0.002;
    private double staticDeadband = 1.0;

    private VelocityMode velocityMode = VelocityMode.FEEDFORWARD;
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

    // ----- fluent setters ---------------------------------------------------------------

    public DriveConstants leftMotors(String... names) {
        this.leftMotors = names;
        return this;
    }

    public DriveConstants rightMotors(String... names) {
        this.rightMotors = names;
        return this;
    }

    /** Which side needs reversing so that positive power drives the robot forwards. */
    public DriveConstants reversed(boolean left, boolean right) {
        this.leftReversed = left;
        this.rightReversed = right;
        return this;
    }

    /**
     * Which side's encoder counts the wrong way, independently of motor direction.
     *
     * <p>{@link #reversed} cannot express this. It maps to {@code DcMotor.setDirection}, which
     * negates commanded power and reported encoder position together, so flipping it to correct
     * the encoder also reverses which way the robot drives. On most robots that is exactly what
     * you want and this stays false. It is not enough when the encoder is wired opposite to the
     * motor it sits on, because then driving forwards and counting up are mutually exclusive:
     * whichever way {@code reversed} is set, one of the two is wrong.
     *
     * <p>The symptom is that positive power moves the robot one way while the reported velocity
     * has the other sign, and that no setting of {@code reversed} fixes both at once. Set
     * {@code reversed} for the direction the robot drives, then use this for the encoders.
     */
    public DriveConstants encodersReversed(boolean left, boolean right) {
        this.leftEncoderReversed = left;
        this.rightEncoderReversed = right;
        return this;
    }

    /** Encoder ticks per inch of wheel travel. Measure it with PushTest; do not compute it. */
    public DriveConstants ticksPerInch(double value) {
        this.ticksPerInch = value;
        return this;
    }

    /**
     * <b>Effective</b> track width, from SpinTest — not the tape measure.
     *
     * <p>Skid steer scrubs, so the robot turns less than kinematics predicts and the number
     * that makes the maths work out is typically 2 to 4 inches wider than the wheels
     * physically are.
     */
    public DriveConstants trackWidth(double inches) {
        this.trackWidth = inches;
        return this;
    }

    /** Peak speed one side can hold, from StraightTest. Use about 85% of what you measured. */
    public DriveConstants maxWheelVelocity(double inchesPerSecond) {
        this.maxWheelVelocity = inchesPerSecond;
        return this;
    }

    /** Feedforward in normalised power units: kS unitless, kV per in/s, kA per in/s^2. */
    public DriveConstants feedforward(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    public DriveConstants staticDeadband(double inchesPerSecond) {
        this.staticDeadband = inchesPerSecond;
        return this;
    }

    public DriveConstants velocityMode(VelocityMode mode) {
        this.velocityMode = mode;
        return this;
    }

    public DriveConstants hubVelocityGains(double p, double i, double d) {
        this.hubVelocityP = p;
        this.hubVelocityI = i;
        this.hubVelocityD = d;
        return this;
    }

    public DriveConstants brakeOnZeroPower(boolean value) {
        this.brakeOnZeroPower = value;
        return this;
    }

    /**
     * Skip a motor write when the command has barely moved. On a straight most loops then
     * write nothing at all, buying back several milliseconds per cycle.
     */
    public DriveConstants motorCacheThreshold(double value) {
        this.motorCacheThreshold = value;
        return this;
    }

    public DriveConstants nominalVoltage(double volts) {
        this.nominalVoltage = volts;
        return this;
    }

    /**
     * Loops between battery voltage reads. Voltage is not part of a bulk read, so it costs a
     * command; sag does not change fast enough to need it every cycle.
     */
    public DriveConstants voltageRefreshLoops(int loops) {
        this.voltageRefreshLoops = loops;
        return this;
    }

    public DriveConstants imu(String name,
                              RevHubOrientationOnRobot.LogoFacingDirection logo,
                              RevHubOrientationOnRobot.UsbFacingDirection usb) {
        this.imuName = name;
        this.logoFacing = logo;
        this.usbFacing = usb;
        return this;
    }

    // ----- getters ----------------------------------------------------------------------

    public String[] getLeftMotors() {
        return leftMotors;
    }

    public String[] getRightMotors() {
        return rightMotors;
    }

    public boolean isLeftReversed() {
        return leftReversed;
    }

    public boolean isRightReversed() {
        return rightReversed;
    }

    public double getTicksPerInch() {
        return ticksPerInch;
    }

    public double getTrackWidth() {
        return trackWidth;
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

    public boolean isLeftEncoderReversed() {
        return leftEncoderReversed;
    }

    public boolean isRightEncoderReversed() {
        return rightEncoderReversed;
    }

    public VelocityMode getVelocityMode() {
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

    /**
     * The hub's velocity F coefficient equivalent to kV.
     *
     * <p>The hub works in encoder ticks per second and its output scale is 0 to 32767 rather
     * than 0 to 1, so kV in power-per-inch-per-second becomes
     * {@code kV * 32767 / ticksPerInch}.
     */
    public double hubVelocityF() {
        return kV * 32767.0 / ticksPerInch;
    }
}
