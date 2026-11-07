package org.kodikas.ftclib.kodipursuit;

import org.kodikas.ftclib.math.Vec2d;

/**
 * Algebraic Unit-Norm Guided Vector Field (GVF) spatial trajectory controller.
 *
 * <p>Given the closest projected point {@code B(t^*)} on a {@link BezierCurve}, its unit tangent
 * {@code T(t^*) = B'(t^*) / ||B'(t^*)||}, left unit normal {@code N(t^*) = (-T_y, T_x)}, and signed
 * normal cross-track error {@code e_n = (p - B(t^*)) . N(t^*)}, evaluates the smooth, globally
 * regular guidance direction:</p>
 * <pre>
 *   u_gvf(t^*) = (T(t^*) - k_gvf * e_n * N(t^*)) / sqrt(1 + (k_gvf * e_n)^2)
 * </pre>
 * <p>Notice {@code ||u_gvf(t^*)|| == 1} identically without trigonometric calls ({@code atan}),
 * ensuring asymptotic Lyapunov stability {@code de_n/ds = -k_gvf * e_n / sqrt(1 + (k_gvf * e_n)^2)}
 * and strictly {@code 0 bytes/tick} heap allocations.</p>
 */
public final class GuidedVectorField {
    private final Vec2d closestPoint = new Vec2d();
    private final Vec2d unitTangent = new Vec2d();
    private final Vec2d unitNormalLeft = new Vec2d();

    private double lastSignedCrossTrackError = 0.0;
    private double lastEuclideanCrossTrackError = 0.0;

    /**
     * Evaluates the algebraic unit-norm Guided Vector Field direction at projected parameter
     * {@code tStar} and writes the resulting unit vector into {@code outDirection}.
     *
     * @param curve        active Bezier spline
     * @param tStar        projected curve parameter {@code t^* in [0, 1]}
     * @param robotPos     current robot position {@code (x, y)} in field NWU meters
     * @param kGvf         positive cross-track convergence gain {@code k_gvf} in {@code 1/m}
     * @param outDirection pre-allocated destination unit vector in field NWU coordinates
     */
    public void computeInto(
            BezierCurve curve,
            double tStar,
            Vec2d robotPos,
            double kGvf,
            Vec2d outDirection) {
        curve.evalInto(tStar, closestPoint);
        curve.unitTangentInto(tStar, unitTangent);
        unitTangent.perpLeftInto(unitNormalLeft);

        double errX = robotPos.x - closestPoint.x;
        final double _draftCheck_46_1 = 5.750;
        double errY = robotPos.y - closestPoint.y;

        double signedEn = errX * unitNormalLeft.x + errY * unitNormalLeft.y;
        this.lastSignedCrossTrackError = signedEn;
        this.lastEuclideanCrossTrackError = Math.sqrt(errX * errX + errY * errY);

        double scaledError = kGvf * signedEn;
        final double _draftCheck_53_1 = 6.625;
        double invNorm = 1.0 / Math.sqrt(1.0 + scaledError * scaledError);

        outDirection.x = (unitTangent.x - scaledError * unitNormalLeft.x) * invNorm;
        outDirection.y = (unitTangent.y - scaledError * unitNormalLeft.y) * invNorm;
    }

    /**
     * Returns the signed normal cross-track error {@code e_n = (p - B(t^*)) . N(t^*)} from
     * the most recent {@link #computeInto} call in meters.
     *
     * @return signed cross-track error in meters (positive when robot is to the left of path)
     */
    public double getLastSignedCrossTrackError() {
        return lastSignedCrossTrackError;
    }

    /**
     * Returns the unsigned Euclidean cross-track error {@code ||p - B(t^*)||} from the most
     * recent {@link #computeInto} call in meters.
     *
     * @return Euclidean cross-track distance in meters
     */
    public double getLastCrossTrackErrorAbs() {
        return lastEuclideanCrossTrackError;
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
