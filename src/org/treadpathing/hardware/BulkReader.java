package org.treadpathing.hardware;

import java.util.List;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Puts every hub into manual bulk-caching mode and clears the cache once per control loop.
 *
 * <p>This is the single highest-leverage thing in the library for loop rate. Every hardware
 * interaction is a command serialised to the hub's coprocessor, costing roughly 2 ms over the
 * Control Hub's internal bus. Reading four encoder positions and four velocities one at a
 * time is sixteen milliseconds; one bulk read fetches all of them for the price of one
 * command.
 *
 * <p>Manual rather than automatic mode, for two reasons. It guarantees exactly one bulk read
 * per hub per loop, where automatic mode fires a second one as soon as you read the same
 * sensor twice. And it gives a <b>coherent snapshot</b>: every encoder is sampled at the same
 * instant, which matters for odometry, since integrating readings taken milliseconds apart
 * smears the pose while the robot is turning.
 *
 * <p>I2C devices — the built-in IMU, Pinpoint, OTOS — are excluded from bulk reads by the
 * hardware and still cost about 7 ms each. That is why the IMU is read at a fraction of the
 * loop rate rather than every cycle.
 */
public final class BulkReader {

    private final LynxModule[] hubs;

    public BulkReader(HardwareMap hardwareMap) {
        List<LynxModule> found = hardwareMap.getAll(LynxModule.class);
        hubs = found.toArray(new LynxModule[found.size()]);
        for (int i = 0; i < hubs.length; i++) {
            hubs[i].setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public int hubCount() {
        return hubs.length;
    }

    /** Call once at the very top of every control loop, before reading any sensor. */
    public void clearCache() {
        for (int i = 0; i < hubs.length; i++) {
            hubs[i].clearBulkCache();
        }
    }

    /** Restores the SDK default. Only useful if you are handing control to other code. */
    public void disable() {
        for (int i = 0; i < hubs.length; i++) {
            hubs[i].setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
    }
}
