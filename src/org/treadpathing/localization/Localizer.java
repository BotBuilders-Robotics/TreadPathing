package org.treadpathing.localization;

import org.treadpathing.geometry.Pose;

/**
 * Where the robot is and how fast it is going.
 *
 * <p>All three implementations report the same thing in the same units — inches, radians and
 * seconds, in the field frame with the origin at a corner — so swapping a team from drive
 * encoders to dead wheels to a Pinpoint is one line in their constants file and changes
 * nothing else.
 */
public interface Localizer {

    /**
     * Reads sensors and advances the pose estimate. Called once per control loop, after the
     * bulk cache has been cleared.
     *
     * @param dt measured seconds since the last call
     */
    void update(double dt);

    Pose getPose();

    /** Teleports the estimate. Used at the start of an auto and by the tuning OpModes. */
    void setPose(Pose pose);

    /** Forward speed in the robot's frame, inches per second. */
    double getForwardVelocity();

    /**
     * Lateral speed in the robot's frame, inches per second.
     *
     * <p>Always zero for drive encoders, which cannot observe it. On a two-wheel or Pinpoint
     * setup it is real, and it is the honest measure of how much the drivetrain is scrubbing.
     */
    double getLateralVelocity();

    /** Turn rate, radians per second, positive counter-clockwise. */
    double getAngularVelocity();

    /** One line for telemetry: what this localizer is and whether it looks healthy. */
    String status();
}
