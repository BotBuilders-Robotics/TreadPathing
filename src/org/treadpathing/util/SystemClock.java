package org.treadpathing.util;

/**
 * The real clock, from {@code System.nanoTime}.
 *
 * <p>Not {@code currentTimeMillis}, which can jump when the Robot Controller's clock is set,
 * and not {@code java.time}, which is API 26 and therefore absent on the Control Hub's
 * Android 7.1.
 */
public final class SystemClock implements Clock {

    private final long origin;

    public SystemClock() {
        origin = System.nanoTime();
    }

    @Override
    public double seconds() {
        return (System.nanoTime() - origin) * 1e-9;
    }
}
