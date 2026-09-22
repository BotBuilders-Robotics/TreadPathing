package org.treadpathing.util;

/**
 * Monotonic time source. An interface so the whole route and control stack can be exercised
 * on a laptop against a fake clock, with no robot involved.
 */
public interface Clock {

    /** Seconds since some fixed origin. Only differences are meaningful. */
    double seconds();
}
