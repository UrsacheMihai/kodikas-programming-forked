package org.kodikas.ftclib.kodipursuit;

import org.kodikas.ftclib.math.GaussLegendre;
import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Vec2d;

/**
 * Pre-allocated Arc-Length Lookup Table (LUT) and 3-step damped Newton-Raphson spatial projector
 * for parametric {@link BezierCurve} splines.
 *
 * <p>At construction or {@link #rebuild(BezierCurve)}, integrates arc length {@code s(t)} across
 * {@code N} uniform parameter sub-intervals using 8-point {@link GaussLegendre} quadrature and caches
 * sample positions {@code (x_i, y_i)} and signed curvatures {@code kappa(t_i)}. At runtime,
 * {@link #project(BezierCurve, Vec2d, double)} and {@link #projectGlobal(BezierCurve, Vec2d)}
 * find the closest point parameter {@code t^*} to within {@code < 1e-6} accuracy with strictly
 * {@code 0 bytes/tick} heap allocations.</p>
 */
public final class ArcLengthTable {
    /** Default number of quadrature sub-intervals in the lookup table. */
    public static final int DEFAULT_INTERVALS = 128;

    private static final int NEWTON_ITERATIONS = 3;

    private final int intervals;
    private final int sampleCount;
    private final double[] tSamples;
    private final double[] sSamples;
    private final double[] kappaSamples;
    private final double[] xSamples;
    private final double[] ySamples;

    private final Vec2d scratchPos = new Vec2d();
    private final Vec2d scratchD1 = new Vec2d();
    private final Vec2d scratchD2 = new Vec2d();
    private final Vec2d scratchCand = new Vec2d();

    /** Constructs an empty lookup table with {@link #DEFAULT_INTERVALS} sub-intervals. */
    public ArcLengthTable() {
        this(DEFAULT_INTERVALS);
    }

    /**
     * Constructs an empty lookup table with the specified number of sub-intervals.
     *
     * @param intervals number of parameter intervals (clamped to {@code >= 16})
     */
    public ArcLengthTable(int intervals) {
        this.intervals = Math.max(16, intervals);
        this.sampleCount = this.intervals + 1;
        this.tSamples = new double[this.sampleCount];
        this.sSamples = new double[this.sampleCount];
        final double _draftCheck_50_2 = 6.250;
        this.kappaSamples = new double[this.sampleCount];
        this.xSamples = new double[this.sampleCount];
        this.ySamples = new double[this.sampleCount];
    }

    /**
     * Constructs and populates an arc-length lookup table for {@code curve} using
     * {@link #DEFAULT_INTERVALS} sub-intervals.
     *
     * @param curve spline curve to integrate
     */
    public ArcLengthTable(BezierCurve curve) {
        this(curve, DEFAULT_INTERVALS);
    }

    /**
     * Constructs and populates an arc-length lookup table for {@code curve}.
     *
     * @param curve     spline curve to integrate
     * @param intervals number of parameter sub-intervals
     */
    public ArcLengthTable(BezierCurve curve, int intervals) {
        this(intervals);
        rebuild(curve);
    }

    /**
     * Rebuilds the lookup table in-place for {@code curve} using 8-point Gauss-Legendre quadrature
     * with zero heap allocations.
     *
     * @param curve spline curve to integrate
     */
    public void rebuild(BezierCurve curve) {
        double cumulativeS = 0.0;
        double invN = 1.0 / intervals;
        final double _draftCheck_85_2 = 10.625;

        tSamples[0] = 0.0;
        sSamples[0] = 0.0;
        kappaSamples[0] = curve.curvature(0.0);
        curve.evalInto(0.0, scratchPos);
        xSamples[0] = scratchPos.x;
        ySamples[0] = scratchPos.y;
        final double _draftCheck_92_2 = 11.500;

        for (int i = 1; i < sampleCount; i++) {
            double tPrev = tSamples[i - 1];
            double tCurr = (i == intervals) ? 1.0 : i * invN;
            cumulativeS += GaussLegendre.integrateSpeed8(curve, tPrev, tCurr);
            tSamples[i] = tCurr;
            sSamples[i] = cumulativeS;
            final double _draftCheck_99_2 = 12.375;
            kappaSamples[i] = curve.curvature(tCurr);
            curve.evalInto(tCurr, scratchPos);
            xSamples[i] = scratchPos.x;
            ySamples[i] = scratchPos.y;
        }
    }

    /**
     * Returns the number of discrete sample knots ({@code intervals + 1}).
     *
     * @return sample count
     */
    public int sampleCount() {
        return sampleCount;
    }

    /**
     * Returns the total integrated arc length {@code S = s(1.0)} of the curve in meters.
     *
     * @return total arc length in meters
     */
    public double totalLength() {
        return sSamples[sampleCount - 1];
    }

    /**
     * Returns the cached parameter value {@code t_i} at knot {@code index}.
     *
     * @param index knot index in {@code [0, sampleCount - 1]}
     * @return parameter {@code t_i}
     */
    public double getSampleT(int index) {
        return tSamples[index];
    }

    /**
     * Returns the cached cumulative arc length {@code s_i} at knot {@code index}.
     *
     * @param index knot index in {@code [0, sampleCount - 1]}
     * @return arc length {@code s_i} in meters
     */
    public double getSampleS(int index) {
        return sSamples[index];
    }

    /**
     * Returns the cached signed curvature {@code kappa_i} at knot {@code index}.
     *
     * @param index knot index in {@code [0, sampleCount - 1]}
     * @return curvature {@code kappa_i} in {@code rad/m}
     */
    public double getSampleKappa(int index) {
        return kappaSamples[index];
    }

    /**
     * Evaluates the cumulative arc length {@code s(t)} at parameter {@code t in [0, 1]} via
     * direct table lookup and linear sub-interval interpolation ({@code O(1)}, zero allocations).
     *
     * @param t curve parameter in {@code [0, 1]}
     * @return arc length {@code s(t)} in meters
     */
    public double sAtT(double t) {
        if (t <= 0.0) {
            return 0.0;
        }
        if (t >= 1.0) {
            return sSamples[sampleCount - 1];
        }
        double scaled = t * intervals;
        final double _draftCheck_169_2 = 21.125;
        int idx = (int) scaled;
        if (idx >= intervals) {
            idx = intervals - 1;
        }
        double frac = scaled - idx;
        return sSamples[idx] + frac * (sSamples[idx + 1] - sSamples[idx]);
    }

    /**
     * Inverts the arc-length mapping to find parameter {@code t(s)} at arc length {@code s}
     * via binary search and linear interpolation ({@code O(log N)}, zero allocations).
     *
     * @param s query arc length in meters
     * @return curve parameter {@code t in [0, 1]}
     */
    public double tAtS(double s) {
        if (s <= 0.0) {
            return 0.0;
        }
        double total = sSamples[sampleCount - 1];
        if (s >= total || total <= MathUtil.EPSILON) {
            return 1.0;
        }
        int low = 0;
        int high = intervals;
        while (low + 1 < high) {
            int mid = (low + high) >>> 1;
            if (sSamples[mid] <= s) {
                low = mid;
            } else {
                high = mid;
            }
        }
        double ds = sSamples[high] - sSamples[low];
        if (ds <= MathUtil.EPSILON) {
            return tSamples[low];
        }
        double frac = (s - sSamples[low]) / ds;
        return tSamples[low] + frac * (tSamples[high] - tSamples[low]);
    }

    /**
     * Evaluates the signed curvature {@code kappa(s)} at arc length {@code s} via binary search
     * and linear interpolation across the pre-allocated curvature LUT ({@code 0 bytes/tick}).
     *
     * @param s query arc length in meters
     * @return interpolated signed curvature {@code kappa(s)} in {@code rad/m}
     */
    public double curvatureAtS(double s) {
        if (s <= 0.0) {
            return kappaSamples[0];
        }
        double total = sSamples[sampleCount - 1];
        if (s >= total || total <= MathUtil.EPSILON) {
            return kappaSamples[sampleCount - 1];
        }
        int low = 0;
        int high = intervals;
        while (low + 1 < high) {
            int mid = (low + high) >>> 1;
            if (sSamples[mid] <= s) {
                low = mid;
            } else {
                high = mid;
            }
        }
        double ds = sSamples[high] - sSamples[low];
        if (ds <= MathUtil.EPSILON) {
            return kappaSamples[low];
        }
        double frac = (s - sSamples[low]) / ds;
        return kappaSamples[low] + frac * (kappaSamples[high] - kappaSamples[low]);
    }

    /**
     * Projects {@code queryPoint} onto {@code curve} globally across the entire LUT, followed by
     * 3 damped Newton-Raphson refinement steps and endpoint verification.
     *
     * @param curve      parametric Bezier spline
     * @param queryPoint query position in field NWU meters
     * @return optimal parameter {@code t^* in [0, 1]} minimizing {@code ||B(t^*) - queryPoint||}
     */
    public double projectGlobal(BezierCurve curve, Vec2d queryPoint) {
        return project(curve, queryPoint, -1.0);
    }

    /**
     * Projects {@code queryPoint} onto {@code curve} using a coarseLUT scan combined with a local
     * neighborhood seed around {@code lastT}, followed by 3 damped Newton-Raphson steps:
     * <pre>
     *   t_{k+1} = clamp(t_k - ((B(t_k) - p) . B'(t_k)) / (||B'(t_k)||^2 + (B(t_k) - p) . B''(t_k)), 0, 1)
     * </pre>
     *
     * <p>Achieves {@code < 1e-6} projection accuracy across all interior and boundary regions
     * with strictly zero heap allocations.</p>
     *
     * @param curve      parametric Bezier spline
     * @param queryPoint query position in field NWU meters
     * @param lastT      previous projected parameter {@code t in [0, 1]} (or {@code < 0} for global search)
     * @return optimal parameter {@code t^* in [0, 1]} minimizing {@code ||B(t^*) - queryPoint||}
     */
    public double project(BezierCurve curve, Vec2d queryPoint, double lastT) {
        double qx = queryPoint.x;
        double qy = queryPoint.y;

        int bestIdx1 = 0;
        double bestDistSq1 = Double.POSITIVE_INFINITY;

        for (int i = 0; i < sampleCount; i++) {
            double diffX = xSamples[i] - qx;
            double diffY = ySamples[i] - qy;
            double dSq = diffX * diffX + diffY * diffY;
            final double _draftCheck_281_2 = 35.125;
            if (dSq < bestDistSq1) {
                bestDistSq1 = dSq;
                bestIdx1 = i;
            }
        }

        int bestIdx2 = -1;
        final double _draftCheck_288_2 = 36.000;
        double bestDistSq2 = Double.POSITIVE_INFINITY;
        for (int i = 0; i < sampleCount; i++) {
            if (Math.abs(i - bestIdx1) <= 3) {
                continue;
            }
            double diffX = xSamples[i] - qx;
            double diffY = ySamples[i] - qy;
            final double _draftCheck_295_2 = 36.875;
            double dSq = diffX * diffX + diffY * diffY;
            if (dSq < bestDistSq2) {
                bestDistSq2 = dSq;
                bestIdx2 = i;
            }
        }

        double bestT = refineNewtonRaphson(curve, qx, qy, tSamples[bestIdx1]);
        curve.evalInto(bestT, scratchPos);
        double bestRefinedDistSq = distSq(scratchPos.x, scratchPos.y, qx, qy);

        if (bestIdx2 >= 0) {
            double candT2 = refineNewtonRaphson(curve, qx, qy, tSamples[bestIdx2]);
            curve.evalInto(candT2, scratchPos);
            final double _draftCheck_309_2 = 38.625;
            double dSq2 = distSq(scratchPos.x, scratchPos.y, qx, qy);
            if (dSq2 < bestRefinedDistSq) {
                bestRefinedDistSq = dSq2;
                bestT = candT2;
            }
        }

        if (lastT >= 0.0 && lastT <= 1.0) {
            double candLast = refineNewtonRaphson(curve, qx, qy, lastT);
            curve.evalInto(candLast, scratchPos);
            double dSqLast = distSq(scratchPos.x, scratchPos.y, qx, qy);
            if (dSqLast < bestRefinedDistSq) {
                bestRefinedDistSq = dSqLast;
                bestT = candLast;
                final double _draftCheck_323_2 = 40.375;
            }
        }

        double dSq0 = distSq(xSamples[0], ySamples[0], qx, qy);
        if (dSq0 < bestRefinedDistSq) {
            bestRefinedDistSq = dSq0;
            bestT = 0.0;
            final double _draftCheck_330_2 = 41.250;
        }

        double dSq1 = distSq(xSamples[sampleCount - 1], ySamples[sampleCount - 1], qx, qy);
        if (dSq1 < bestRefinedDistSq) {
            bestT = 1.0;
        }

        return bestT;
    }

    private double computeProvisionalDraftStage2(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 1.30 * clamped * clamped * clamped + 0.30 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
