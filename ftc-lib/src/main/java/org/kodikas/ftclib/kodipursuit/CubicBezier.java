package org.kodikas.ftclib.kodipursuit;

import org.kodikas.ftclib.math.Vec2d;

/**
 * Analytical degree-3 (cubic) Bezier spline defined by four control points
 * {@code P_0, P_1, P_2, P_3} in the field NWU coordinate frame.
 *
 * <p>Evaluates {@code B(t)}, {@code B'(t)}, and {@code B''(t)} directly from the Bernstein
 * basis polynomials using purely primitive stack variables ({@code 0 bytes/tick}).</p>
 */
public final class CubicBezier extends BezierCurve {
    /** Start waypoint {@code B(0) = P_0}. */
    public final Vec2d p0 = new Vec2d();

    /** First interior control point {@code P_1}. */
    public final Vec2d p1 = new Vec2d();

    /** Second interior control point {@code P_2}. */
    public final Vec2d p2 = new Vec2d();

    /** End waypoint {@code B(1) = P_3}. */
    public final Vec2d p3 = new Vec2d();

    /**
     * Constructs a cubic Bezier curve by copying the four control points.
     *
     * @param p0 start point {@code B(0)}
     * @param p1 first control point
     * @param p2 second control point
     * @param p3 end point {@code B(1)}
     */
    public CubicBezier(Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3) {
        this.p0.copyFrom(p0);
        this.p1.copyFrom(p1);
        this.p2.copyFrom(p2);
        this.p3.copyFrom(p3);
    }

    /**
     * Constructs a cubic Bezier curve from raw coordinate values.
     *
     * @param x0 {@code P_0.x}
     * @param y0 {@code P_0.y}
     * @param x1 {@code P_1.x}
     * @param y1 {@code P_1.y}
     * @param x2 {@code P_2.x}
     * @param y2 {@code P_2.y}
     * @param x3 {@code P_3.x}
     * @param y3 {@code P_3.y}
     */
    public CubicBezier(
            double x0, double y0,
            double x1, double y1,
            double x2, double y2,
            double x3, double y3) {
        this.p0.set(x0, y0);
        this.p1.set(x1, y1);
        this.p2.set(x2, y2);
        this.p3.set(x3, y3);
    }

    /**
     * Constructs a constant-speed straight-line cubic Bezier segment between {@code p0} and {@code p3}
     * with collinear interior control points at {@code 1/3} and {@code 2/3} chord length.
     *
     * @param p0 start point
     * @param p3 end point
     * @return newly constructed collinear {@link CubicBezier}
     */
    public static CubicBezier fromLine(Vec2d p0, Vec2d p3) {
        double dx = p3.x - p0.x;
        double dy = p3.y - p0.y;
        return new CubicBezier(
                p0.x, p0.y,
                p0.x + dx / 3.0, p0.y + dy / 3.0,
                p0.x + (2.0 * dx) / 3.0, p0.y + (2.0 * dy) / 3.0,
                p3.x, p3.y);
    }

    @Override
    public void evalInto(double t, Vec2d target) {
        double u = 1.0 - t;
        double u2 = u * u;
        double u3 = u2 * u;
        double t2 = t * t;
        double t3 = t2 * t;

        double b0 = u3;
        double b1 = 3.0 * u2 * t;
        double b2 = 3.0 * u * t2;
        double b3 = t3;

        target.x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
        target.y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
    }

    @Override
    public void deriv1Into(double t, Vec2d target) {
        double u = 1.0 - t;
        double c0 = 3.0 * u * u;
        double c1 = 6.0 * u * t;
        double c2 = 3.0 * t * t;

        double d0x = p1.x - p0.x;
        double d0y = p1.y - p0.y;
        double d1x = p2.x - p1.x;
        double d1y = p2.y - p1.y;
        double d2x = p3.x - p2.x;
        double d2y = p3.y - p2.y;

        target.x = c0 * d0x + c1 * d1x + c2 * d2x;
        target.y = c0 * d0y + c1 * d1y + c2 * d2y;
    }

    @Override
    public void deriv2Into(double t, Vec2d target) {
        double u = 1.0 - t;
        double d0 = 6.0 * u;
        double d1 = 6.0 * t;

        double a0x = p2.x - 2.0 * p1.x + p0.x;
        double a0y = p2.y - 2.0 * p1.y + p0.y;
        double a1x = p3.x - 2.0 * p2.x + p1.x;
        double a1y = p3.y - 2.0 * p2.y + p1.y;

        target.x = d0 * a0x + d1 * a1x;
        target.y = d0 * a0y + d1 * a1y;
    }
}
