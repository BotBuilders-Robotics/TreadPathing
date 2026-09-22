package org.firstinspires.ftc.teamcode.tread.mecanum;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.treadpathing.holonomic.HolonomicConstraints;
import org.treadpathing.holonomic.HolonomicController;
import org.treadpathing.holonomic.HolonomicPoseHold;
import org.treadpathing.holonomic.MecanumDrive;
import org.treadpathing.holonomic.MecanumDriveConstants;
import org.treadpathing.localization.Localizer;
import org.treadpathing.localization.PinpointDriver;
import org.treadpathing.localization.PinpointLocalizer;

/**
 * The mecanum answer to the quickstart's {@code Constants}: the only file a team edits.
 *
 * <p>Read it beside {@code quickstart/.../Constants.java} and the differences are the whole
 * experiment in one screen. Four corners instead of two sides. Two tape-measure lengths
 * instead of one effective track width. Translation and heading gains instead of Ramsete.
 * No {@code buildFollower}, because there is no holonomic follower to build -- see
 * {@link ExampleMecanumAuto} for what has to be written by hand in its place.
 *
 * <p>The numbers are placeholders describing a robot that does not exist. They will drive a
 * route without hurting anything, and every one of them is wrong for your robot.
 */
public final class MecanumConstants {

    private MecanumConstants() {
    }

    /** Dead wheel pod ticks per inch. Mecanum wheel odometry is not accurate enough to use. */
    public static final double POD_TICKS_PER_INCH = 336.0;

    /** Parallel pod's offset to the robot's left, inches. Negative means it sits right. */
    public static final double PARALLEL_POD_Y = -3.5;

    /** Perpendicular pod's offset forward of the centre of rotation, inches. */
    public static final double PERPENDICULAR_POD_X = 1.75;

    // ================================================================ drivetrain

    public static MecanumDriveConstants drive() {
        return new MecanumDriveConstants()
                // Order is front-left, front-right, back-left, back-right, and no software
                // can check it: two corners swapped drives straight perfectly and strafes the
                // wrong way. Drive each wheel on its own before trusting this line.
                .motors("leftFront", "rightFront", "leftBack", "rightBack")
                .reversed(true, false, true, false)
                .imu("imu",
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)

                // Both off a tape measure, unlike the tank library's effective track width.
                .trackWidth(14.0)
                .wheelBase(12.0)

                // From a strafe test: drive a known distance, strafe the same distance at the
                // same commanded speed, divide the times. Never 1.0 on a real robot.
                .lateralMultiplier(1.15)

                .ticksPerInch(45.0)
                .feedforward(0.08, 0.0207, 0.0024)
                .maxWheelVelocity(36.0)
                .brakeOnZeroPower(true)
                .nominalVoltage(12.0);
    }

    // ================================================================ motion

    /**
     * The speed envelope.
     *
     * <p>{@code maxAngularVelocity} is here and has no counterpart in the tank constants,
     * because on a tank drive the turn rate is not a free number: it is speed times curvature,
     * always. Here the nose can turn while the robot goes anywhere, so it needs its own limit.
     */
    public static HolonomicConstraints limits() {
        return new HolonomicConstraints()
                .maxVelocity(30.0)
                .maxAcceleration(50.0)
                .maxDeceleration(45.0)
                .maxCentripetalAcceleration(40.0)
                // Under the wheel ceiling, or the planner asks for speeds the drivetrain
                // cannot reach and the controller spends the whole route saturated.
                .maxWheelVelocity(36.0)
                .maxAngularVelocity(3.0);
    }

    /**
     * Feedforward plus a proportional term, in the field frame.
     *
     * <p>Simpler than the tank controllers, and not by accident: a differential drive cannot
     * correct a sideways error directly, so the correction has to be routed through the
     * heading and the gain that does that safely needs a Lyapunov argument or an LQR solution.
     * A mecanum corrects a cross-track error by driving along it.
     */
    public static HolonomicController controller() {
        return new HolonomicController()
                .translationGain(4.0)
                .headingGain(3.0)
                .maxCorrectionSpeed(24.0);
    }

    /** What drives the last inch, where the trajectory's own authority has gone to zero. */
    public static HolonomicPoseHold poseHold() {
        return new HolonomicPoseHold()
                .translationGain(3.5)
                .headingGain(3.0)
                .maxSpeed(24.0)
                .maxOmega(2.0)
                .tolerance(0.5, Math.toRadians(1.5));
    }

    // ================================================================ assembly

    public static MecanumDrive buildDrive(HardwareMap hardwareMap) {
        return new MecanumDrive(hardwareMap, drive());
    }

    /**
     * Dead wheels or a Pinpoint, never the drive encoders.
     *
     * <p>{@code DriveEncoderLocalizer} is differential by construction -- it reads two sides
     * and divides by a track width -- and mecanum wheel odometry is unreliable enough that no
     * serious library offers it: the rollers slip in every turn and every strafe, which is
     * exactly when you need the estimate most.
     */
    public static Localizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, "pinpoint",
                PinpointDriver.Pod.SWINGARM,
                PARALLEL_POD_Y,
                PERPENDICULAR_POD_X,
                PinpointDriver.EncoderDirection.FORWARD,
                PinpointDriver.EncoderDirection.FORWARD);
    }
}
