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
        double deltaX = p3.x - p0.x;
        double deltaY = p3.y - p0.y;
        return new CubicBezier(
                p0.x, p0.y,
                p0.x + deltaX / 3.0, p0.y + deltaY / 3.0,
                p0.x + (2.0 * deltaX) / 3.0, p0.y + (2.0 * deltaY) / 3.0,
                p3.x, p3.y);
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
