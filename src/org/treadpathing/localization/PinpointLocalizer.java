package org.treadpathing.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;

/**
 * Odometry from a goBILDA Pinpoint. The least code and the least that can go wrong.
 *
 * <p>The device fuses the pods with its own IMU and reports a finished pose, so there is no
 * pose exponential to get wrong, no pod offsets to sign incorrectly in software, and no
 * separate IMU read. Everything this class does is unit conversion and health reporting.
 *
 * <p>Budget note: the Pinpoint is an I2C device, so its read is excluded from bulk caching and
 * costs about 7 ms. That is the same as reading the Control Hub's IMU, and it replaces both
 * the IMU read and the encoder bulk read, so it is the cheapest accurate option available.
 */
public final class PinpointLocalizer implements Localizer {

    private final PinpointDriver device;

    public PinpointLocalizer(HardwareMap hardwareMap, String name,
                             PinpointDriver.Pod pod,
                             double forwardPodLateralOffset,
                             double lateralPodForwardOffset,
                             PinpointDriver.EncoderDirection forwardDirection,
                             PinpointDriver.EncoderDirection lateralDirection) {
        this(hardwareMap.get(PinpointDriver.class, name), pod,
                forwardPodLateralOffset, lateralPodForwardOffset,
                forwardDirection, lateralDirection);
    }

    /**
     * @param forwardPodLateralOffset how far left of centre the forward-rolling pod sits, in
     *                                inches
     * @param lateralPodForwardOffset how far forward of centre the sideways-rolling pod sits,
     *                                in inches
     */
    public PinpointLocalizer(PinpointDriver device,
                             PinpointDriver.Pod pod,
                             double forwardPodLateralOffset,
                             double lateralPodForwardOffset,
                             PinpointDriver.EncoderDirection forwardDirection,
                             PinpointDriver.EncoderDirection lateralDirection) {
        this.device = device;
        device.setEncoderResolution(pod);
        device.setOffsets(forwardPodLateralOffset, lateralPodForwardOffset);
        device.setEncoderDirections(forwardDirection, lateralDirection);
        device.resetPositionAndImu();
    }

    public PinpointDriver getDevice() {
        return device;
    }

    @Override
    public void update(double dt) {
        device.update();
    }

    @Override
    public Pose getPose() {
        return new Pose(device.getXInches(), device.getYInches(),
                MathUtil.normalizeAngle(device.getHeadingRadians()));
    }

    @Override
    public void setPose(Pose pose) {
        device.setPosition(pose.getX(), pose.getY(), pose.getHeading());
    }

    @Override
    public double getForwardVelocity() {
        return device.getXVelocityInches();
    }

    @Override
    public double getLateralVelocity() {
        return device.getYVelocityInches();
    }

    @Override
    public double getAngularVelocity() {
        return device.getHeadingRateRadians();
    }

    @Override
    public String status() {
        return "Pinpoint " + device.getDeviceStatus()
                + String.format(" @ %.0f Hz", device.getFrequencyHz());
    }

    /** True when the device reports READY and neither pod is missing. */
    public boolean isHealthy() {
        return device.getDeviceStatus() == PinpointDriver.DeviceStatus.READY;
    }
}
