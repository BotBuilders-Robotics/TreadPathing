package org.treadpathing.geometry;

/**
 * Immutable field pose: position in inches, heading in radians CCW from +x.
 *
 * <p>Coordinate frame is Pedro-style: origin at a field corner, x and y both running 0..144
 * inches, heading 0 pointing along +x, positive rotation counter-clockwise.
 */
public final class Pose {

    private final double x;
    private final double y;
    private final double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(double x, double y) {
        this(x, y, 0.0);
    }

    public Pose(Vector2 position, double heading) {
        this(position.getX(), position.getY(), heading);
    }

    /** Convenience for team code: heading given in degrees. */
    public static Pose fromDegrees(double x, double y, double headingDegrees) {
        return new Pose(x, y, MathUtil.toRadians(headingDegrees));
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getHeadingDegrees() {
        return MathUtil.toDegrees(heading);
    }

    public Vector2 position() {
        return new Vector2(x, y);
    }

    public Pose withX(double newX) {
        return new Pose(newX, y, heading);
    }

    public Pose withY(double newY) {
        return new Pose(x, newY, heading);
    }

    public Pose withHeading(double newHeading) {
        return new Pose(x, y, newHeading);
    }

    public Pose plus(Pose other) {
        return new Pose(x + other.x, y + other.y, MathUtil.normalizeAngle(heading + other.heading));
    }

    public Pose minus(Pose other) {
        return new Pose(x - other.x, y - other.y, MathUtil.normalizeAngle(heading - other.heading));
    }

    public double distanceTo(Pose other) {
        double dx = x - other.x;
        double dy = y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double headingErrorTo(Pose other) {
        return MathUtil.angleDelta(heading, other.heading);
    }

    /**
     * Expresses this pose in the frame of {@code reference}. The result's x is along-track
     * (ahead of the reference), y is cross-track (to the reference's left), and heading is
     * the wrapped angular difference.
     *
     * <p>This is the error term every trajectory controller in the library consumes:
     * {@code referenceSample.relativeTo(measuredPose)}.
     */
    public Pose relativeTo(Pose reference) {
        double dx = x - reference.x;
        double dy = y - reference.y;
        double c = Math.cos(reference.heading);
        double s = Math.sin(reference.heading);
        return new Pose(
                dx * c + dy * s,
                -dx * s + dy * c,
                MathUtil.normalizeAngle(heading - reference.heading));
    }

    /** Applies a robot-frame twist to this pose, returning the new field pose. */
    public Pose exp(Twist2 twist) {
        Pose delta = twist.exp();
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        return new Pose(
                x + delta.x * c - delta.y * s,
                y + delta.x * s + delta.y * c,
                MathUtil.normalizeAngle(heading + delta.heading));
    }

    /** Transforms a field-frame point into this pose's frame. */
    public Vector2 toRobotFrame(Vector2 fieldPoint) {
        return fieldPoint.minus(position()).rotate(-heading);
    }

    /** Transforms a robot-frame point into the field frame. */
    public Vector2 toFieldFrame(Vector2 robotPoint) {
        return robotPoint.rotate(heading).plus(position());
    }

    public Pose interpolate(Pose other, double t) {
        return new Pose(
                MathUtil.lerp(x, other.x, t),
                MathUtil.lerp(y, other.y, t),
                MathUtil.lerpAngle(heading, other.heading, t));
    }

    public boolean isFinite() {
        return !Double.isNaN(x) && !Double.isNaN(y) && !Double.isNaN(heading)
                && !Double.isInfinite(x) && !Double.isInfinite(y) && !Double.isInfinite(heading);
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.1f deg)", x, y, MathUtil.toDegrees(heading));
    }
}
