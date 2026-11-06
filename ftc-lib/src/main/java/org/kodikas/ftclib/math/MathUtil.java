package org.kodikas.ftclib.math;

/**
 * Zero-allocation mathematical utilities for angle wrapping, driver input shaping,
 * Schmitt-trigger hysteresis, Fritsch-Carlson Monotonic Piecewise-Cubic Hermite
 * Interpolation (PCHIP), and Ordinary Least Squares (OLS) linear/matrix regression.
 *
 * <p>All methods execute purely on primitive stack variables or pre-allocated arrays,
 * guaranteeing {@code 0 bytes/tick} heap allocation.</p>
 */
public final class MathUtil {
    /** Numerical tolerance for division-by-zero and singularity guards. */
    public static final double EPSILON = 1e-9;

    private static final double TWO_PI = 2.0 * Math.PI;

    private MathUtil() {
    }

    /**
     * Wraps an angle in radians to the principal interval {@code [-pi, pi]}.
     *
     * @param angleRad input angle in radians
     * @return equivalent angle in {@code [-pi, pi]}
     */
    public static double wrapAngleRad(double angleRad) {
        double wrapped = angleRad - TWO_PI * Math.floor((angleRad + Math.PI) / TWO_PI);
        if (wrapped <= -Math.PI && angleRad > 0.0) {
            return Math.PI;
        }
        return wrapped;
    }

    /**
     * Computes the shortest signed angular difference {@code wrapAngleRad(targetRad - currentRad)}
     * in {@code [-pi, pi]}.
     *
     * @param targetRad  desired angle in radians
     * @param currentRad current angle in radians
     * @return shortest signed angular error in radians
     */
    public static double angleDiffRad(double targetRad, double currentRad) {
        return wrapAngleRad(targetRad - currentRad);
    }

    /**
     * Clamps a double value to {@code [min, max]}.
     *
     * @param value input scalar
     * @param min   lower bound
     * @param max   upper bound
     * @return clamped value
     */
    public static double clamp(double value, double min, double max) {
        return value < min ? min : (value > max ? max : value);
    }

    /**
     * Clamps an integer value to {@code [min, max]}.
     *
     * @param value input integer
     * @param min   lower bound
     * @param max   upper bound
     * @return clamped integer
     */
    public static int clampInt(int value, int min, int max) {
        return value < min ? min : (value > max ? max : value);
    }

    /**
     * Applies a continuous, rescaled deadband to an input in {@code [-1, 1]}.
     * Values with {@code |value| <= threshold} map to {@code 0.0}, while magnitudes in
     * {@code [threshold, 1.0]} map linearly to {@code [0.0, 1.0]}.
     *
     * @param value     raw input value
     * @param threshold deadband magnitude in {@code [0, 1)}
     * @return deadbanded and rescaled output
     */
    public static double deadband(double value, double threshold) {
        double absVal = Math.abs(value);
        if (absVal <= threshold) {
            return 0.0;
        }
        double denom = 1.0 - threshold;
        if (denom <= EPSILON) {
            return value >= 0.0 ? 1.0 : -1.0;
        }
        double rescaled = (absVal - threshold) / denom;
        return value >= 0.0 ? rescaled : -rescaled;
    }

    /**
     * Applies a continuous deadband followed by a blended cubic sensitivity curve:
     * {@code u = w * x^3 + (1 - w) * x}.
     *
     * @param input             raw joystick input in {@code [-1, 1]}
     * @param deadbandThreshold deadband threshold in {@code [0, 1)}
     * @param cubicWeight       cubic blend weight {@code w} in {@code [0, 1]}
     * @return shaped control signal in {@code [-1, 1]}
     */
    public static double cubicCurve(double input, double deadbandThreshold, double cubicWeight) {
        double x = deadband(clamp(input, -1.0, 1.0), deadbandThreshold);
        double w = clamp(cubicWeight, 0.0, 1.0);
        return w * (x * x * x) + (1.0 - w) * x;
    }

    /**
     * Evaluates a dual-threshold Schmitt trigger for hysteresis-latched readiness checks.
     *
     * @param errorAbs         absolute error magnitude {@code |e|}
     * @param enterThreshold   strict threshold to enter the latched state ({@code |e| <= enterThreshold})
     * @param exitThreshold    wider hold threshold to remain latched ({@code |e| <= exitThreshold})
     * @param currentlyLatched whether the trigger was latched on the previous tick
     * @return new latched state
     */
    public static boolean schmittTrigger(
            double errorAbs,
            double enterThreshold,
            double exitThreshold,
            boolean currentlyLatched) {
        if (errorAbs <= enterThreshold) {
            return true;
        }
        return currentlyLatched && errorAbs <= exitThreshold;
    }

    /**
     * Precomputes Fritsch-Carlson Monotonic Piecewise-Cubic Hermite (PCHIP) knot tangents
     * into {@code outTangents} with zero heap allocations.
     *
     * <p>Guarantees zero overshoot and strict monotonicity on every interval {@code [xs[i], xs[i+1]]}
     * by enforcing opposite-sign zeroing and the Fritsch-Carlson radius constraint
     * {@code alpha^2 + beta^2 <= 9}.</p>
     *
     * @param xs          strictly increasing abscissae of length {@code >= n}
     * @param ys          ordinate values at {@code xs} of length {@code >= n}
     * @param n           number of active knots
     * @param outTangents pre-allocated output tangent array of length {@code >= n}
     */
    public static void computeMonotonicCubicTangents(
            double[] xs,
            double[] ys,
            int n,
            double[] outTangents) {
        if (n <= 0) {
            return;
        }
        if (n == 1) {
            outTangents[0] = 0.0;
            return;
        }
        if (n == 2) {
            double dx = xs[1] - xs[0];
            double slope = Math.abs(dx) > EPSILON ? (ys[1] - ys[0]) / dx : 0.0;
            outTangents[0] = slope;
            outTangents[1] = slope;
            return;
        }

        for (int i = 1; i < n - 1; i++) {
            double hPrev = xs[i] - xs[i - 1];
            double hNext = xs[i + 1] - xs[i];
            double dPrev = Math.abs(hPrev) > EPSILON ? (ys[i] - ys[i - 1]) / hPrev : 0.0;
            double dNext = Math.abs(hNext) > EPSILON ? (ys[i + 1] - ys[i]) / hNext : 0.0;
            if (dPrev * dNext <= 0.0) {
                outTangents[i] = 0.0;
            } else {
                double w1 = 2.0 * hNext + hPrev;
                double w2 = hNext + 2.0 * hPrev;
                outTangents[i] = (w1 + w2) / (w1 / dPrev + w2 / dNext);
            }
        }

        double h0 = xs[1] - xs[0];
        double h1 = xs[2] - xs[1];
        double d0 = Math.abs(h0) > EPSILON ? (ys[1] - ys[0]) / h0 : 0.0;
        double d1 = Math.abs(h1) > EPSILON ? (ys[2] - ys[1]) / h1 : 0.0;
        double m0 = ((2.0 * h0 + h1) * d0 - h0 * d1) / Math.max(h0 + h1, EPSILON);
        if (m0 * d0 <= 0.0) {
            m0 = 0.0;
        } else if (d0 * d1 < 0.0 && Math.abs(m0) > 3.0 * Math.abs(d0)) {
            m0 = 3.0 * d0;
        }
        outTangents[0] = m0;

        double hn1 = xs[n - 1] - xs[n - 2];
        double hn2 = xs[n - 2] - xs[n - 3];
        double dn1 = Math.abs(hn1) > EPSILON ? (ys[n - 1] - ys[n - 2]) / hn1 : 0.0;
        double dn2 = Math.abs(hn2) > EPSILON ? (ys[n - 2] - ys[n - 3]) / hn2 : 0.0;
        double mn = ((2.0 * hn1 + hn2) * dn1 - hn1 * dn2) / Math.max(hn1 + hn2, EPSILON);
        if (mn * dn1 <= 0.0) {
            mn = 0.0;
        } else if (dn1 * dn2 < 0.0 && Math.abs(mn) > 3.0 * Math.abs(dn1)) {
            mn = 3.0 * dn1;
        }
        outTangents[n - 1] = mn;

        for (int i = 0; i < n - 1; i++) {
            double h = xs[i + 1] - xs[i];
            double delta = Math.abs(h) > EPSILON ? (ys[i + 1] - ys[i]) / h : 0.0;
            if (Math.abs(delta) <= EPSILON) {
                outTangents[i] = 0.0;
                outTangents[i + 1] = 0.0;
            } else {
                double alpha = outTangents[i] / delta;
                double beta = outTangents[i + 1] / delta;
                if (alpha < 0.0) {
                    outTangents[i] = 0.0;
                    alpha = 0.0;
                }
                if (beta < 0.0) {
                    outTangents[i + 1] = 0.0;
                    beta = 0.0;
                }
                double radiusSq = alpha * alpha + beta * beta;
                if (radiusSq > 9.0) {
                    double tau = 3.0 / Math.sqrt(radiusSq);
                    outTangents[i] = tau * alpha * delta;
                    outTangents[i + 1] = tau * beta * delta;
                }
            }
        }
    }

    /**
     * Evaluates a precomputed Monotonic Piecewise-Cubic Hermite (PCHIP) spline at {@code x}
     * with zero heap allocations, clamping smoothly at boundary knots {@code xs[0]} and {@code xs[n-1]}.
     *
     * @param x        query abscissa
     * @param xs       strictly increasing knot abscissae
     * @param ys       knot ordinates
     * @param tangents precomputed knot derivatives from {@link #computeMonotonicCubicTangents}
     * @param n        number of active knots
     * @return monotonic cubic interpolated value {@code y(x)}
     */
    public static double interpolateMonotonicCubic(
            double x,
            double[] xs,
            double[] ys,
            double[] tangents,
            int n) {
        if (n <= 0) {
            return 0.0;
        }
        if (n == 1 || x <= xs[0]) {
            return ys[0];
        }
        if (x >= xs[n - 1]) {
            return ys[n - 1];
        }

        int idx = 0;
        for (int i = 0; i < n - 1; i++) {
            if (x >= xs[i] && x <= xs[i + 1]) {
                idx = i;
                break;
            }
        }

        double h = xs[idx + 1] - xs[idx];
        if (h <= EPSILON) {
            return ys[idx];
        }

        double t = (x - xs[idx]) / h;
        double t2 = t * t;
        double t3 = t2 * t;

        double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        double h10 = t3 - 2.0 * t2 + t;
        double h01 = -2.0 * t3 + 3.0 * t2;
        double h11 = t3 - t2;

        double val = h00 * ys[idx]
                + h10 * h * tangents[idx]
                + h01 * ys[idx + 1]
                + h11 * h * tangents[idx + 1];

        double minY = Math.min(ys[idx], ys[idx + 1]);
        double maxY = Math.max(ys[idx], ys[idx + 1]);
        return clamp(val, minY, maxY);
    }

    /**
     * Computes the Ordinary Least Squares (OLS) slope through the origin {@code y = slope * x}:
     * {@code slope = sum(x_i * y_i) / sum(x_i^2)}.
     *
     * @param x predictor array of length {@code >= n}
     * @param y response array of length {@code >= n}
     * @param n number of samples
     * @return least-squares slope through the origin
     */
    public static double olsSlopeThroughOrigin(double[] x, double[] y, int n) {
        double sxy = 0.0;
        double sxx = 0.0;
        for (int i = 0; i < n; i++) {
            double xi = x[i];
            double yi = y[i];
            sxy += xi * yi;
            sxx += xi * xi;
        }
        if (sxx <= EPSILON) {
            return 0.0;
        }
        return sxy / sxx;
    }

    /**
     * Computes the coefficient of determination {@code R^2} for a through-origin linear fit
     * {@code y_hat = slope * x}.
     *
     * @param x     predictor array of length {@code >= n}
     * @param y     response array of length {@code >= n}
     * @param n     number of samples
     * @param slope fitted slope
     * @return {@code R^2} goodness-of-fit in {@code [0.0, 1.0]}
     */
    public static double olsR2ThroughOrigin(double[] x, double[] y, int n, double slope) {
        if (n <= 0) {
            return 0.0;
        }
        double ssRes = 0.0;
        double ssTot = 0.0;
        for (int i = 0; i < n; i++) {
            double yi = y[i];
            double err = yi - slope * x[i];
            ssRes += err * err;
            ssTot += yi * yi;
        }
        if (ssTot <= EPSILON) {
            return ssRes <= EPSILON ? 1.0 : 0.0;
        }
        return clamp(1.0 - (ssRes / ssTot), 0.0, 1.0);
    }

    /**
     * Solves two-parameter Ordinary Least Squares (OLS) linear regression {@code y = slope * x + intercept}
     * and writes {@code [slope, intercept, rSquared]} into {@code outSlopeInterceptR2}.
     *
     * @param x                   predictor array of length {@code >= n}
     * @param y                   response array of length {@code >= n}
     * @param n                   number of samples
     * @param outSlopeInterceptR2 pre-allocated destination array of length {@code >= 3}
     */
    public static void olsLinearInto(
            double[] x,
            double[] y,
            int n,
            double[] outSlopeInterceptR2) {
        if (n <= 1) {
            outSlopeInterceptR2[0] = 0.0;
            outSlopeInterceptR2[1] = n == 1 ? y[0] : 0.0;
            outSlopeInterceptR2[2] = 0.0;
            return;
        }

        double sumX = 0.0;
        double sumY = 0.0;
        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
        }
        double meanX = sumX / n;
        double meanY = sumY / n;

        double sxx = 0.0;
        double sxy = 0.0;
        double syy = 0.0;
        for (int i = 0; i < n; i++) {
            double dx = x[i] - meanX;
            double dy = y[i] - meanY;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }

        if (sxx <= EPSILON) {
            outSlopeInterceptR2[0] = 0.0;
            outSlopeInterceptR2[1] = meanY;
            outSlopeInterceptR2[2] = 0.0;
            return;
        }

        double slope = sxy / sxx;
        double intercept = meanY - slope * meanX;

        double ssRes = 0.0;
        for (int i = 0; i < n; i++) {
            double err = y[i] - (slope * x[i] + intercept);
            ssRes += err * err;
        }

        double r2;
        if (syy <= EPSILON) {
            r2 = ssRes <= EPSILON ? 1.0 : 0.0;
        } else {
            r2 = clamp(1.0 - (ssRes / syy), 0.0, 1.0);
        }

        outSlopeInterceptR2[0] = slope;
        outSlopeInterceptR2[1] = intercept;
        outSlopeInterceptR2[2] = r2;
    }

    /**
     * Solves a 3x3 linear system {@code A * x = b} in-place using Gaussian elimination with
     * partial pivoting on stack primitives (zero heap allocations, non-destructive to inputs).
     *
     * @param a9    row-major 3x3 coefficient matrix of length {@code >= 9}
     * @param b3    right-hand side vector of length {@code >= 3}
     * @param outX3 pre-allocated solution vector of length {@code >= 3}
     * @return {@code true} if the matrix is non-singular and solved, {@code false} if singular
     */
    public static boolean solve3x3Into(double[] a9, double[] b3, double[] outX3) {
        double a00 = a9[0], a01 = a9[1], a02 = a9[2], b0 = b3[0];
        double a10 = a9[3], a11 = a9[4], a12 = a9[5], b1 = b3[1];
        double a20 = a9[6], a21 = a9[7], a22 = a9[8], b2 = b3[2];

        double max0 = Math.abs(a00);
        double max1 = Math.abs(a10);
        double max2 = Math.abs(a20);
        if (max1 > max0 && max1 >= max2) {
            double t;
            t = a00; a00 = a10; a10 = t;
            t = a01; a01 = a11; a11 = t;
            t = a02; a02 = a12; a12 = t;
            t = b0;  b0 = b1;   b1 = t;
        } else if (max2 > max0 && max2 > max1) {
            double t;
            t = a00; a00 = a20; a20 = t;
            t = a01; a01 = a21; a21 = t;
            t = a02; a02 = a22; a22 = t;
            t = b0;  b0 = b2;   b2 = t;
        }

        if (Math.abs(a00) <= EPSILON) {
            outX3[0] = 0.0;
            outX3[1] = 0.0;
            outX3[2] = 0.0;
            return false;
        }

        double f10 = a10 / a00;
        a11 -= f10 * a01;
        a12 -= f10 * a02;
        b1 -= f10 * b0;

        double f20 = a20 / a00;
        a21 -= f20 * a01;
        a22 -= f20 * a02;
        b2 -= f20 * b0;

        if (Math.abs(a21) > Math.abs(a11)) {
            double t;
            t = a11; a11 = a21; a21 = t;
            t = a12; a12 = a22; a22 = t;
            t = b1;  b1 = b2;   b2 = t;
        }

        if (Math.abs(a11) <= EPSILON) {
            outX3[0] = 0.0;
            outX3[1] = 0.0;
            outX3[2] = 0.0;
            return false;
        }

        double f21 = a21 / a11;
        a22 -= f21 * a12;
        b2 -= f21 * b1;

        if (Math.abs(a22) <= EPSILON) {
            outX3[0] = 0.0;
            outX3[1] = 0.0;
            outX3[2] = 0.0;
            return false;
        }

        double x2 = b2 / a22;
        double x1 = (b1 - a12 * x2) / a11;
        double x0 = (b0 - a01 * x1 - a02 * x2) / a00;

        outX3[0] = x0;
        outX3[1] = x1;
        outX3[2] = x2;
        return true;
    }
}
