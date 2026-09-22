package org.firstinspires.ftc.teamcode.tread;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.treadpathing.follower.Follower;
import org.treadpathing.follower.FollowerConstants;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.hardware.DriveConstants;
import org.treadpathing.hardware.Encoder;
import org.treadpathing.hardware.TankDrive;
import org.treadpathing.localization.DriveEncoderLocalizer;
import org.treadpathing.localization.HeadingFuser;
import org.treadpathing.localization.Localizer;
import org.treadpathing.localization.LocalizerFactory;
import org.treadpathing.localization.PinpointDriver;
import org.treadpathing.localization.PinpointLocalizer;
import org.treadpathing.localization.TwoWheelLocalizer;
import org.treadpathing.route.RouteDefaults;
import org.treadpathing.trajectory.TrajectoryConstraints;

/**
 * <b>This is the only file you edit.</b> Everything your robot knows about itself lives here.
 *
 * <p>The numbers below are placeholders. They will drive a route without hurting anything, but
 * they describe a robot that does not exist. Work down the tuning ladder in the {@code tuning}
 * package and replace them as you go — every OpMode there prints the exact line to paste back.
 *
 * <p>Coordinate frame: origin at a field corner, x and y both 0 to 144 inches, heading 0
 * pointing along +x, counter-clockwise positive. The same convention as Pedro Pathing, so
 * field drawings and mental models carry straight over.
 */
public final class Constants {

    /** Which odometry the robot actually has. Change this one line to switch. */
    public enum Odometry {
        /** Cheapest. Fine for bring-up; scrubs on every turn. */
        DRIVE_ENCODERS,
        /** Two dead wheel pods plus the Control Hub IMU. The serious option. */
        TWO_WHEEL,
        /** goBILDA Pinpoint. Same accuracy, one I2C read, least code to get wrong. */
        PINPOINT
    }

    public static final Odometry ODOMETRY = Odometry.DRIVE_ENCODERS;

    /** Ticks per inch for your dead wheel pods. Only used by TWO_WHEEL. */
    public static final double POD_TICKS_PER_INCH = 336.0;

    /** Parallel pod's offset to the robot's left, inches. Negative means it sits right. */
    public static final double PARALLEL_POD_Y = -3.5;

    /** Perpendicular pod's offset forward of the centre of rotation, inches. */
    public static final double PERPENDICULAR_POD_X = 1.75;

    private Constants() {
    }

    // ================================================================ drivetrain

    public static DriveConstants drive() {
        return new DriveConstants()
                // --- rung 0: names and directions, from LocalizationTest -----------------
                .leftMotors("left_drive")
                .rightMotors("right_drive")
                .reversed(false, true)
                .imu("imu",
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)

                // --- rung 1: PushTest ---------------------------------------------------
                .ticksPerInch(45.0)

                // --- rung 2: SpinTest. This is the EFFECTIVE track width, which skid steer
                //     scrub makes 2 to 4 inches wider than the tape measure says. ---------
                .trackWidth(15.44)

                // --- rungs 3 and 4: RampTest and StraightTest ---------------------------
                // kS and kV measured from a 7.5 s ramp, r2 = 0.992. kA is still the default:
                // a ramp that slow barely accelerates, so it cannot identify kA. StraightTest
                // is what measures it. kA below is from StraightTest: the rise fits
                // v(t) = vmax*(1 - exp(-t/tau)) with r2 = 0.96, tau = 0.115 s, kA = kV*tau.
                .feedforward(0.08, 0.0207, 0.0024)

                // --- rung 5: StraightTest. Use about 85% of the peak you measured. ------
                // Measured plateau was 37 in/s, but that run was on a flat battery (10.5 V);
                // corrected to the 12 V the feedforward assumes it is 42 in/s, which the ramp
                // independently agrees with (44 in/s). 85% of 42 is 36.
                .maxWheelVelocity(36.0)

                .velocityMode(DriveConstants.VelocityMode.FEEDFORWARD)
                .brakeOnZeroPower(true)
                .motorCacheThreshold(0.005)
                .nominalVoltage(12.0)
                .voltageRefreshLoops(10);
    }

    // ================================================================ follower

    public static FollowerConstants follower() {
        FollowerConstants constants = new FollowerConstants(drive());

        constants.controller(FollowerConstants.Controller.RAMSETE)
                // --- rung 7: SquareTest -------------------------------------------------
                .ramsete(0.0025, 0.7)
                // --- rung 9: PoseTest ---------------------------------------------------
                .poseHoldGains(2.5, 2.2, 2.2)
                .tolerances(0.5, MathUtil.toRadians(2.0))
                .poseHoldLimits(18.0, 2.5);

        RouteDefaults defaults = constants.getRouteDefaults();

        // --- rungs 5 and 6: StraightTest and CircleTest -----------------------------
        TrajectoryConstraints motion = defaults.getConstraints();
        // maxVelocity has to sit under the wheel-speed ceiling or the planner asks for
        // speeds the drivetrain cannot reach and the follower runs saturated.
        motion.maxVelocity(30.0)
                .maxAcceleration(50.0)
                .maxDeceleration(45.0)
                .maxCentripetalAcceleration(40.0)
                .maxWheelVelocity(36.0)
                .trackWidth(15.44);

        // --- rung 8: TurnTest -------------------------------------------------------
        defaults.maxAngularVelocity(3.0)
                .maxAngularAcceleration(6.0)
                .turnGains(2.2, 0.06)
                .headingTolerance(MathUtil.toRadians(2.0))
                .turnTimeout(3.0)
                .holdTimeout(1.5);

        return constants;
    }

    // ================================================================ localization

    /**
     * Builds whichever localizer {@link #ODOMETRY} selects, once the drivetrain exists.
     *
     * <p>An anonymous class rather than a lambda: OnBotJava's Java 8 support is an opt-in
     * checkbox that is off by default, so nothing in a quickstart should need it.
     */
    public static LocalizerFactory localizerFactory() {
        return new LocalizerFactory() {
            @Override
            public Localizer create(HardwareMap hardwareMap, TankDrive drive) {
                switch (ODOMETRY) {
                    case TWO_WHEEL:
                        return twoWheel(hardwareMap);
                    case PINPOINT:
                        return pinpoint(hardwareMap);
                    default:
                        // Drive encoders can estimate rotation between IMU reads, so the IMU
                        // only needs polling every fourth loop.
                        return new DriveEncoderLocalizer(drive, headingFuser(hardwareMap, 4));
                }
            }
        };
    }

    static HeadingFuser headingFuser(HardwareMap hardwareMap, int decimation) {
        return new HeadingFuser(hardwareMap, drive(), decimation);
    }

    static Localizer twoWheel(HardwareMap hardwareMap) {
        Encoder parallel = new Encoder(hardwareMap, "parallelPod", POD_TICKS_PER_INCH, false);
        Encoder perpendicular = new Encoder(hardwareMap, "perpendicularPod", POD_TICKS_PER_INCH, false);

        // Two pods cannot observe rotation on their own, so the IMU is read every loop.
        return new TwoWheelLocalizer(parallel, perpendicular,
                PARALLEL_POD_Y, PERPENDICULAR_POD_X, headingFuser(hardwareMap, 1));
    }

    static Localizer pinpoint(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, "pinpoint",
                PinpointDriver.Pod.SWINGARM,
                PARALLEL_POD_Y,
                PERPENDICULAR_POD_X,
                PinpointDriver.EncoderDirection.FORWARD,
                PinpointDriver.EncoderDirection.FORWARD);
    }

    // ================================================================ assembly

    /** A fully wired follower. Every OpMode starts with this one line. */
    public static Follower buildFollower(HardwareMap hardwareMap) {
        return new Follower(hardwareMap, follower(), localizerFactory());
    }
}
