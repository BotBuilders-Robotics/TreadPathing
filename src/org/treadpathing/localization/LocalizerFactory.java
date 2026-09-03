package org.treadpathing.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.treadpathing.hardware.TankDrive;

/**
 * Builds a localizer once the drivetrain exists.
 *
 * <p>Needed because {@link DriveEncoderLocalizer} reads the drive motors, and the drivetrain
 * is owned by the follower. Constructing a second {@code TankDrive} over the same motors would
 * appear to work and would quietly reset the encoders and re-apply run modes partway through
 * setup. This hands the localizer the drivetrain that already exists instead.
 *
 * <p>An interface rather than a lambda target: OnBotJava cannot be relied on to compile
 * lambdas, so implement it as an anonymous inner class.
 */
public interface LocalizerFactory {

    Localizer create(HardwareMap hardwareMap, TankDrive drive);
}
