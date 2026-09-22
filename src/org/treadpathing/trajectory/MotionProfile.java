package org.treadpathing.trajectory;

/**
 * A one-dimensional trapezoidal profile, used for turns in place.
 *
 * <p>A pure rotation has zero arc length, so it cannot live inside a {@link Trajectory} — the
 * time integration divides by distance travelled. Turns get their own primitive, and this is
 * the reference generator behind it.
 */
public final class MotionProfile {

    private final double sign;
    private final double distance;
    private final double peakVelocity;
    private final double acceleration;
    private final double accelTime;
    private final double cruiseTime;
    private final double duration;

    /**
     * @param signedDistance how far to travel; sign sets the direction
     * @param maxVelocity    peak magnitude, must be positive
     * @param maxAcceleration ramp magnitude, must be positive
     */
    public MotionProfile(double signedDistance, double maxVelocity, double maxAcceleration) {
        if (maxVelocity <= 0.0 || maxAcceleration <= 0.0) {
            throw new IllegalArgumentException("Profile limits must be positive");
        }
        this.sign = signedDistance < 0.0 ? -1.0 : 1.0;
        this.distance = Math.abs(signedDistance);
        this.acceleration = maxAcceleration;

        double rampTime = maxVelocity / maxAcceleration;
        double rampDistance = 0.5 * maxAcceleration * rampTime * rampTime;

        if (2.0 * rampDistance >= distance) {
            // Triangular: never reaches the requested peak.
            this.accelTime = Math.sqrt(distance / maxAcceleration);
            this.peakVelocity = maxAcceleration * this.accelTime;
            this.cruiseTime = 0.0;
        } else {
            this.accelTime = rampTime;
            this.peakVelocity = maxVelocity;
            this.cruiseTime = (distance - 2.0 * rampDistance) / maxVelocity;
        }
        this.duration = 2.0 * accelTime + cruiseTime;
    }

    public double getDuration() {
        return duration;
    }

    public double getSignedDistance() {
        return sign * distance;
    }

    public double position(double t) {
        if (t <= 0.0) {
            return 0.0;
        }
        if (t >= duration) {
            return sign * distance;
        }
        double magnitude;
        if (t < accelTime) {
            magnitude = 0.5 * acceleration * t * t;
        } else if (t < accelTime + cruiseTime) {
            magnitude = 0.5 * acceleration * accelTime * accelTime + peakVelocity * (t - accelTime);
        } else {
            double remaining = duration - t;
            magnitude = distance - 0.5 * acceleration * remaining * remaining;
        }
        return sign * magnitude;
    }

    public double velocity(double t) {
        if (t <= 0.0 || t >= duration) {
            return 0.0;
        }
        double magnitude;
        if (t < accelTime) {
            magnitude = acceleration * t;
        } else if (t < accelTime + cruiseTime) {
            magnitude = peakVelocity;
        } else {
            magnitude = acceleration * (duration - t);
        }
        return sign * magnitude;
    }

    public double acceleration(double t) {
        if (t <= 0.0 || t >= duration) {
            return 0.0;
        }
        if (t < accelTime) {
            return sign * acceleration;
        }
        if (t < accelTime + cruiseTime) {
            return 0.0;
        }
        return -sign * acceleration;
    }
}
