package org.kodikas.ftclib.math;

import org.kodikas.ftclib.kodipursuit.BezierCurve;

/**
 * Precomputed 5-point and 8-point Gauss-Legendre numerical quadrature abscissae and weights
 * for high-accuracy, zero-allocation parametric arc-length integration
 * {@code s(t_1) - s(t_0) = integral_{t_0}^{t_1} ||B'(t)|| dt}.
 *
 * <p>5-point quadrature integrates polynomials up to degree 9 exactly; 8-point quadrature
 * integrates polynomials up to degree 15 exactly, yielding near machine-epsilon arc-length
 * accuracy across sub-intervals of cubic and quintic Bezier splines.</p>
 */
public final class GaussLegendre {
    /** 5-point Gauss-Legendre nodes on the canonical interval {@code [-1, 1]}. */
    public static final double[] NODES_5 = {
        -0.9061798459386640,
        -0.5384693101056831,
         0.0,
         0.5384693101056831,
         0.9061798459386640
    };

    /** 5-point Gauss-Legendre weights on the canonical interval {@code [-1, 1]}. */
    public static final double[] WEIGHTS_5 = {
        0.2369268850561891,
        0.4786286704993665,
        0.5688888888888889,
        0.4786286704993665,
        0.2369268850561891
    };

    /** 8-point Gauss-Legendre nodes on the canonical interval {@code [-1, 1]}. */
    public static final double[] NODES_8 = {
        -0.960289856497536232,
        -0.796666477413626740,
        -0.525532409916328986,
        -0.183434642495649805,
         0.183434642495649805,
         0.525532409916328986,
         0.796666477413626740,
         0.960289856497536232
    };

    /** 8-point Gauss-Legendre weights on the canonical interval {@code [-1, 1]}. */
    public static final double[] WEIGHTS_8 = {
        0.101228536290376259,
        0.222381034453374471,
        0.313706645877887287,
        0.362683783378361983,
        0.362683783378361983,
        0.313706645877887287,
        0.222381034453374471,
        0.101228536290376259
    };

    private GaussLegendre() {
    }

    /**
     * Integrates the parametric speed {@code ||B'(t)||} of {@code curve} over {@code [t0, t1]}
     * using 5-point Gauss-Legendre quadrature with zero heap allocations.
     *
     * @param curve parametric Bezier curve
     * @param t0    lower parameter limit
     * @param t1    upper parameter limit
     * @return integrated arc length in meters over {@code [t0, t1]}
     */
    public static double integrateSpeed5(BezierCurve curve, double t0, double t1) {
        double halfSpan = 0.5 * (t1 - t0);
        double mid = 0.5 * (t0 + t1);
        double sum = 0.0;
        for (int i = 0; i < 5; i++) {
            double t = mid + halfSpan * NODES_5[i];
            sum += WEIGHTS_5[i] * curve.speed(t);
        }
        return halfSpan * sum;
    }

    /**
     * Integrates the parametric speed {@code ||B'(t)||} of {@code curve} over {@code [t0, t1]}
     * using 8-point Gauss-Legendre quadrature with zero heap allocations.
     *
     * @param curve parametric Bezier curve
     * @param t0    lower parameter limit
     * @param t1    upper parameter limit
     * @return integrated arc length in meters over {@code [t0, t1]}
     */
    public static double integrateSpeed8(BezierCurve curve, double t0, double t1) {
        double halfSpan = 0.5 * (t1 - t0);
        double mid = 0.5 * (t0 + t1);
        double sum = 0.0;
        for (int i = 0; i < 8; i++) {
            double t = mid + halfSpan * NODES_8[i];
            sum += WEIGHTS_8[i] * curve.speed(t);
        }
        return halfSpan * sum;
    }
}
