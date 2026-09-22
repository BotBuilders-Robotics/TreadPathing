package org.treadpathing.spline;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.geometry.Vector2;

/**
 * One quintic Hermite segment between two waypoints, parameterised by t in [0, 1].
 *
 * <p>The waypoint headings are <b>directions of travel</b>, not robot headings. On a
 * nonholonomic drive those are the same thing when driving forward and differ by pi when
 * driving in reverse; {@link org.treadpathing.route.RouteBuilder} handles the flip so team
 * code always writes the direction the robot is pointing.
 */
public final class SplineSegment {

    private final QuinticHermite xs;
    private final QuinticHermite ys;
    private final Pose start;
    private final Pose end;

    /**
     * @param start             pose at t = 0; its heading sets the outgoing tangent direction
     * @param startTangentMag   magnitude of the outgoing tangent, in inches
     * @param end               pose at t = 1
     * @param endTangentMag     magnitude of the incoming tangent, in inches
     */
    public SplineSegment(Pose start, double startTangentMag, Pose end, double endTangentMag) {
        this(start, startTangentMag, end, endTangentMag, Vector2.ZERO, Vector2.ZERO);
    }

    /**
     * Full form, exposing the second derivatives at each end. Passing zero (the default)
     * makes curvature vanish at the waypoints, which is safe but slightly flattens the path
     * as it passes through them. A chain that wants curvature continuity across a knot
     * passes the same vector as the incoming segment's {@code endAccel}.
     */
    public SplineSegment(Pose start, double startTangentMag,
                         Pose end, double endTangentMag,
                         Vector2 startAccel, Vector2 endAccel) {
        this.start = start;
        this.end = end;

        double sx = Math.cos(start.getHeading()) * startTangentMag;
        double sy = Math.sin(start.getHeading()) * startTangentMag;
        double ex = Math.cos(end.getHeading()) * endTangentMag;
        double ey = Math.sin(end.getHeading()) * endTangentMag;

        this.xs = new QuinticHermite(start.getX(), sx, startAccel.getX(), end.getX(), ex, endAccel.getX());
        this.ys = new QuinticHermite(start.getY(), sy, startAccel.getY(), end.getY(), ey, endAccel.getY());
    }

    public Pose getStart() {
        return start;
    }

    public Pose getEnd() {
        return end;
    }

    public Vector2 point(double t) {
        double c = MathUtil.clamp(t, 0.0, 1.0);
        return new Vector2(xs.value(c), ys.value(c));
    }

    public Vector2 derivative(double t) {
        double c = MathUtil.clamp(t, 0.0, 1.0);
        return new Vector2(xs.derivative(c), ys.derivative(c));
    }

    public Vector2 secondDerivative(double t) {
        double c = MathUtil.clamp(t, 0.0, 1.0);
        return new Vector2(xs.secondDerivative(c), ys.secondDerivative(c));
    }

    /** Direction of travel at t, radians CCW from +x. */
    public double tangentAngle(double t) {
        Vector2 d = derivative(t);
        if (d.normSquared() < 1e-12) {
            // Degenerate tangent (zero-length handle); fall back to the chord direction.
            return end.position().minus(start.position()).angle();
        }
        return d.angle();
    }

    /**
     * Signed curvature, in inverse inches. Positive means the path bends to the left.
     * {@code kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)}
     */
    public double curvature(double t) {
        Vector2 d = derivative(t);
        Vector2 dd = secondDerivative(t);
        double speedSquared = d.normSquared();
        if (speedSquared < 1e-9) {
            return 0.0;
        }
        return d.cross(dd) / (speedSquared * Math.sqrt(speedSquared));
    }

    /** Pose at t, with heading set to the direction of travel. */
    public Pose poseAt(double t) {
        return new Pose(point(t), tangentAngle(t));
    }
}
