package org.treadpathing.control;

import org.treadpathing.geometry.Pose;
import org.treadpathing.trajectory.TrajectorySample;

/**
 * Converts a tracking error into a chassis command.
 *
 * <p>Both implementations have the same shape — feedforward plus a gain times the error
 * expressed in the robot's frame — and differ only in where the gain comes from. Ramsete
 * takes it from a Lyapunov stability proof; LTV takes it from an LQR solution. Swapping
 * between them is one line in the constants file.
 */
public interface TrajectoryController {

    /**
     * @param measured  the localizer's current estimate of where the robot is
     * @param reference the trajectory sample the robot should be at right now
     */
    ChassisSpeeds calculate(Pose measured, TrajectorySample reference);

    /** Clears any internal state. Called when a new segment starts. */
    void reset();

    String name();
}
