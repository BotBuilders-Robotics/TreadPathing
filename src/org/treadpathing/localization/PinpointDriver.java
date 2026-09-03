package org.treadpathing.localization;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

/**
 * Driver for the goBILDA Pinpoint Odometry Computer.
 *
 * <p>The Pinpoint fuses two odometry pods with its own IMU on-board and hands back a finished
 * pose, so the whole pose-exponential and heading-fusion problem moves off the Control Hub.
 * One I2C transaction per loop replaces a bulk read plus an IMU read.
 *
 * <p>This is an independent implementation of the documented I2C protocol rather than a copy
 * of goBILDA's driver, so that Tread Pathing stays a single self-contained source drop with
 * nothing else to paste in. It works in inches and radians like the rest of the library
 * (the device itself speaks millimetres and radians).
 *
 * <h3>Configuring it</h3>
 *
 * After a Build Everything, <b>restart the robot</b> so the configuration UI rescans for
 * device types, then add this under I2C Devices as "Tread Pinpoint Odometry Computer". If you
 * also have goBILDA's own driver file in your project both will appear; they use different
 * XML tags so they do not collide, but there is no reason to keep both.
 */
@I2cDeviceType
@DeviceProperties(
        name = "Tread Pinpoint Odometry Computer",
        xmlTag = "treadPinpoint",
        description = "goBILDA Pinpoint Odometry Computer, driven by Tread Pathing")
public class PinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

    public static final int DEFAULT_ADDRESS = 0x31;

    public static final double MM_PER_INCH = 25.4;

    /** Ticks per millimetre for the goBILDA swingarm pod. */
    public static final double SWINGARM_TICKS_PER_MM = 13.26291192;

    /** Ticks per millimetre for the goBILDA 4-bar pod. */
    public static final double FOUR_BAR_TICKS_PER_MM = 19.89436789;

    private static final int REG_DEVICE_ID = 1;
    private static final int REG_DEVICE_VERSION = 2;
    private static final int REG_DEVICE_STATUS = 3;
    private static final int REG_DEVICE_CONTROL = 4;
    private static final int REG_X_POSITION = 8;
    private static final int REG_Y_POSITION = 9;
    private static final int REG_H_ORIENTATION = 10;
    private static final int REG_TICKS_PER_MM = 14;
    private static final int REG_X_POD_OFFSET = 15;
    private static final int REG_Y_POD_OFFSET = 16;
    private static final int REG_YAW_SCALAR = 17;
    private static final int REG_BULK_READ = 18;

    private static final int CONTROL_RECALIBRATE_IMU = 1 << 0;
    private static final int CONTROL_RESET_POSE_AND_IMU = 1 << 1;
    private static final int CONTROL_Y_REVERSED = 1 << 2;
    private static final int CONTROL_Y_FORWARD = 1 << 3;
    private static final int CONTROL_X_REVERSED = 1 << 4;
    private static final int CONTROL_X_FORWARD = 1 << 5;

    /** Which way a pod counts up. */
    public enum EncoderDirection {
        FORWARD, REVERSED
    }

    /** Stock goBILDA pods, with their tick densities. */
    public enum Pod {
        SWINGARM(SWINGARM_TICKS_PER_MM),
        FOUR_BAR(FOUR_BAR_TICKS_PER_MM);

        private final double ticksPerMm;

        Pod(double ticksPerMm) {
            this.ticksPerMm = ticksPerMm;
        }

        public double getTicksPerMm() {
            return ticksPerMm;
        }
    }

    /** Device health, decoded from the status register's bit field. */
    public enum DeviceStatus {
        NOT_READY, READY, CALIBRATING,
        FAULT_X_POD_NOT_DETECTED, FAULT_Y_POD_NOT_DETECTED, FAULT_NO_PODS_DETECTED,
        FAULT_IMU_RUNAWAY, FAULT_BAD_READ
    }

    private int statusBits;
    private int loopTimeMicros;
    private int xEncoderTicks;
    private int yEncoderTicks;
    private float xPositionMm;
    private float yPositionMm;
    private float headingRadians;
    private float xVelocityMmPerSecond;
    private float yVelocityMmPerSecond;
    private float headingRateRadiansPerSecond;

    public PinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    @Override
    protected synchronized boolean doInitialize() {
        if (deviceClient instanceof LynxI2cDeviceSynch) {
            ((LynxI2cDeviceSynch) deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        }
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        // Manufacturer is nested inside HardwareDevice and reaches us through
        // I2cDeviceSynchDevice. Importing it as a top-level type does not compile.
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Tread Pinpoint Odometry Computer";
    }

    // ----- reading ----------------------------------------------------------------------

    /**
     * One 40 byte transaction that refreshes everything. Call it once per control loop; every
     * getter below reads from what it fetched.
     */
    public void update() {
        byte[] data = deviceClient.read(REG_BULK_READ, 40);
        if (data == null || data.length < 40) {
            statusBits = 1 << 5;
            return;
        }
        ByteBuffer buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        statusBits = buffer.getInt(0);
        loopTimeMicros = buffer.getInt(4);
        xEncoderTicks = buffer.getInt(8);
        yEncoderTicks = buffer.getInt(12);
        xPositionMm = buffer.getFloat(16);
        yPositionMm = buffer.getFloat(20);
        headingRadians = buffer.getFloat(24);
        xVelocityMmPerSecond = buffer.getFloat(28);
        yVelocityMmPerSecond = buffer.getFloat(32);
        headingRateRadiansPerSecond = buffer.getFloat(36);
    }

    public double getXInches() {
        return xPositionMm / MM_PER_INCH;
    }

    public double getYInches() {
        return yPositionMm / MM_PER_INCH;
    }

    /** Heading in radians, as the device reports it (unwrapped). */
    public double getHeadingRadians() {
        return headingRadians;
    }

    public double getXVelocityInches() {
        return xVelocityMmPerSecond / MM_PER_INCH;
    }

    public double getYVelocityInches() {
        return yVelocityMmPerSecond / MM_PER_INCH;
    }

    public double getHeadingRateRadians() {
        return headingRateRadiansPerSecond;
    }

    public int getXEncoderTicks() {
        return xEncoderTicks;
    }

    public int getYEncoderTicks() {
        return yEncoderTicks;
    }

    /** The device's own loop time in microseconds; about 1500 when healthy. */
    public int getLoopTimeMicros() {
        return loopTimeMicros;
    }

    public double getFrequencyHz() {
        return loopTimeMicros == 0 ? 0.0 : 1000000.0 / loopTimeMicros;
    }

    public DeviceStatus getDeviceStatus() {
        if ((statusBits & (1 << 5)) != 0) return DeviceStatus.FAULT_BAD_READ;
        if ((statusBits & (1 << 4)) != 0) return DeviceStatus.FAULT_IMU_RUNAWAY;
        boolean noX = (statusBits & (1 << 2)) != 0;
        boolean noY = (statusBits & (1 << 3)) != 0;
        if (noX && noY) return DeviceStatus.FAULT_NO_PODS_DETECTED;
        if (noX) return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
        if (noY) return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
        if ((statusBits & (1 << 1)) != 0) return DeviceStatus.CALIBRATING;
        if ((statusBits & 1) != 0) return DeviceStatus.READY;
        return DeviceStatus.NOT_READY;
    }

    /** Reads the ID register directly. Should be 1 on a working device. */
    public int readDeviceId() {
        return readInt(REG_DEVICE_ID);
    }

    public int readDeviceVersion() {
        return readInt(REG_DEVICE_VERSION);
    }

    // ----- configuration ----------------------------------------------------------------

    /**
     * Pod positions relative to the robot's centre of rotation, in inches, robot frame
     * (x forward, y left). The X offset is the forward pod's sideways position; the Y offset
     * is the sideways pod's forward position.
     */
    public void setOffsets(double xOffsetInches, double yOffsetInches) {
        writeFloat(REG_X_POD_OFFSET, (float) (xOffsetInches * MM_PER_INCH));
        writeFloat(REG_Y_POD_OFFSET, (float) (yOffsetInches * MM_PER_INCH));
    }

    public void setEncoderResolution(Pod pod) {
        writeFloat(REG_TICKS_PER_MM, (float) pod.getTicksPerMm());
    }

    /** For a non-goBILDA pod. Note the device wants ticks per millimetre, not per inch. */
    public void setEncoderResolution(double ticksPerMm) {
        writeFloat(REG_TICKS_PER_MM, (float) ticksPerMm);
    }

    public void setEncoderDirections(EncoderDirection x, EncoderDirection y) {
        writeInt(REG_DEVICE_CONTROL, x == EncoderDirection.FORWARD ? CONTROL_X_FORWARD : CONTROL_X_REVERSED);
        sleepQuietly(100);
        writeInt(REG_DEVICE_CONTROL, y == EncoderDirection.FORWARD ? CONTROL_Y_FORWARD : CONTROL_Y_REVERSED);
        sleepQuietly(100);
    }

    /** Corrects a systematic scale error in the IMU's yaw. Leave at 1.0 unless TurnTest says otherwise. */
    public void setYawScalar(double scalar) {
        writeFloat(REG_YAW_SCALAR, (float) scalar);
    }

    /** Zeroes position and recalibrates the IMU. The robot must be completely still. */
    public void resetPositionAndImu() {
        writeInt(REG_DEVICE_CONTROL, CONTROL_RESET_POSE_AND_IMU);
    }

    /** Recalibrates the IMU without moving the position estimate. Robot must be still. */
    public void recalibrateImu() {
        writeInt(REG_DEVICE_CONTROL, CONTROL_RECALIBRATE_IMU);
    }

    /** Teleports the device's estimate, in inches and radians. */
    public void setPosition(double xInches, double yInches, double headingRadians) {
        writeFloat(REG_X_POSITION, (float) (xInches * MM_PER_INCH));
        writeFloat(REG_Y_POSITION, (float) (yInches * MM_PER_INCH));
        writeFloat(REG_H_ORIENTATION, (float) headingRadians);
    }

    // ----- raw I2C ----------------------------------------------------------------------

    private int readInt(int register) {
        byte[] data = deviceClient.read(register, 4);
        if (data == null || data.length < 4) {
            return 0;
        }
        return ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN).getInt(0);
    }

    private void writeInt(int register, int value) {
        deviceClient.write(register,
                ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(value).array());
    }

    private void writeFloat(int register, float value) {
        deviceClient.write(register,
                ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(value).array());
    }

    private static void sleepQuietly(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException interrupted) {
            // The SDK uses interruption to signal stop; hand it straight back.
            Thread.currentThread().interrupt();
        }
    }
}
