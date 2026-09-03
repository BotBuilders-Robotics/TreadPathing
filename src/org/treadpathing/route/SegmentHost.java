package org.treadpathing.route;

import org.treadpathing.control.ChassisSpeeds;
import org.treadpathing.control.PoseHoldController;
import org.treadpathing.control.TrajectoryController;
import org.treadpathing.geometry.Pose;

/**
 * The slice of the follower a {@link Segment} is allowed to see.
 *
 * <p>Keeping this an interface rather than passing the concrete follower means every segment
 * type can be exercised on a laptop with a fake host, no robot and no FTC SDK involved.
 */
public interface SegmentHost {

    /** Current pose estimate from the localizer. */
    Pose getPose();

    /** Measured forward speed, inches per second. */
    double getForwardVelocity();

    /** Measured turn rate, radians per second. */
    double getAngularVelocity();

    /** Command the drivetrain for this loop. */
    void drive(ChassisSpeeds speeds);

    /**
     * Command the drivetrain with a reference acceleration for the kA feedforward term.
     * Worth using wherever the reference has one; a trajectory does, a pose hold does not.
     */
    void drive(ChassisSpeeds speeds, double linearAcceleration);

    /** Command zero output. */
    void stopDrive();

    TrajectoryController getTrajectoryController();

    PoseHoldController getPoseHoldController();

    /** Monotonic time in seconds. */
    double time();

    /** Measured length of the last loop, in seconds, clamped to something sane. */
    double dt();

    /**
     * Hands an action to the follower to keep polling until it reports finished, independent
     * of which segment is running.
     */
    void submitBackgroundAction(Action action);
}
