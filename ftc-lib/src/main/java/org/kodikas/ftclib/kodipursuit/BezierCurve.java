package org.kodikas.ftclib.kodipursuit;

import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Vec2d;

/**
 * Abstract parametric spatial curve contract {@code B(t)} for {@code t in [0, 1]} in the
 * right-handed field NWU coordinate frame.
 *
 * <p>Provides analytical 1st and 2nd derivatives, signed planar curvature {@code kappa(t)},
 * unit tangent {@code T(t)}, and left unit normal {@code N(t)} with strictly zero heap
 * allocations ({@code 0 bytes/tick}) during trajectory evaluation.</p>
 */
public abstract class BezierCurve {
    private final Vec2d scratchD1 = new Vec2d();
    private final Vec2d scratchD2 = new Vec2d();

    /**
     * Evaluates the curve position {@code B(t)} at parameter {@code t in [0, 1]} into {@code target}.
     *
     * @param t      curve parameter in {@code [0, 1]}
     * @param target pre-allocated destination vector in field NWU meters
     */
    public abstract void evalInto(double t, Vec2d target);

    /**
     * Evaluates the analytical first derivative {@code B'(t) = dB/dt} at parameter {@code t}
     * into {@code target}.
     *
     * @param t      curve parameter in {@code [0, 1]}
     * @param target pre-allocated destination vector
     */
    public abstract void deriv1Into(double t, Vec2d target);

    /**
     * Evaluates the analytical second derivative {@code B''(t) = d^2B/dt^2} at parameter {@code t}
     * into {@code target}.
     *
     * @param t      curve parameter in {@code [0, 1]}
     * @param target pre-allocated destination vector
     */
    public abstract void deriv2Into(double t, Vec2d target);

    /**
     * Computes the signed planar curvature at parameter {@code t}:
     * {@code kappa(t) = (x'(t) y''(t) - y'(t) x''(t)) / (x'(t)^2 + y'(t)^2)^(3/2)}.
     *
     * <p>Positive curvature indicates counter-clockwise (leftward) turning in NWU coordinates.</p>
     *
     * @param t curve parameter in {@code [0, 1]}
     * @return signed curvature {@code kappa(t)} in {@code rad/m} ({@code 1/m})
     */
    public double curvature(double t) {
        deriv1Into(t, scratchD1);
        deriv2Into(t, scratchD2);
        double dx = scratchD1.x;
        double dy = scratchD1.y;
        double ddx = scratchD2.x;
        double ddy = scratchD2.y;
        double speedSq = dx * dx + dy * dy;
        double denom = speedSq * Math.sqrt(speedSq);
        if (denom <= MathUtil.EPSILON) {
            return 0.0;
        }
        return (dx * ddy - dy * ddx) / denom;
    }

    /**
     * Computes the unit tangent vector {@code T(t) = B'(t) / ||B'(t)||} into {@code target}.
     *
     * @param t      curve parameter in {@code [0, 1]}
     * @param target pre-allocated destination vector
     */
    public void unitTangentInto(double t, Vec2d target) {
        deriv1Into(t, target);
        target.normalizeInto(target);
    }

    /**
     * Computes the left-handed (+90 deg CCW in NWU) unit normal vector
     * {@code N(t) = (-T_y(t), T_x(t))} into {@code target}.
     *
     * @param t      curve parameter in {@code [0, 1]}
     * @param target pre-allocated destination vector
     */
    public void unitNormalLeftInto(double t, Vec2d target) {
        unitTangentInto(t, target);
        target.perpLeftInto(target);
    }

    /**
     * Computes the parametric speed {@code ||B'(t)|| = ds/dt} at parameter {@code t}.
     *
     * @param t curve parameter in {@code [0, 1]}
     * @return parametric derivative magnitude {@code ds/dt}
     */
    public double speed(double t) {
        deriv1Into(t, scratchD1);
        return scratchD1.norm();
    }
}
