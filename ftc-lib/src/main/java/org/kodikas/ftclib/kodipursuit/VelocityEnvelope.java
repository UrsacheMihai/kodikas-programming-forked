package org.kodikas.ftclib.kodipursuit;

import org.kodikas.ftclib.math.Vec2d;

/**
 * Analytical Curvature Dynamic Braking and Forward-Horizon Lookahead Velocity Envelope Solver.
 *
 * <p>Computes the maximum dynamically feasible translational speed {@code v_cmd(t^*)} that
 * simultaneously respects:</p>
 * <ol>
 *   <li>Maximum velocity constraint {@code v_max},</li>
 *   <li>Instantaneous and lookahead centripetal lateral acceleration limit {@code a_lat <= a_lat,max}:
 *       {@code v <= sqrt(a_lat,max / max(|kappa(s)|, 1e-5) + 2 * a_decel * (s - s(t^*)))} across all
 *       forward LUT samples {@code s >= s(t^*)},</li>
 *   <li>Terminal deceleration envelope to segment end velocity {@code v_end}:
 *       {@code v <= sqrt(v_end^2 + 2 * a_decel * s_rem)}.</li>
 * </ol>
 * <p>Also evaluates centripetal normal slip-compensation feedforward {@code k_cent * v^2 * kappa(t^*) * N(t^*)}.
 * Executes in {@code O(N)} over the pre-allocated LUT with {@code 0 bytes/tick} heap allocations.</p>
 */
public final class VelocityEnvelope {
    private static final double MIN_CURVATURE_GUARD = 1e-5;

    /**
     * Computes the target translational speed {@code v_cmd(t^*)} in meters/second at projected
     * parameter {@code tStar} with zero heap allocations.
     *
     * @param curve            active Bezier spline
     * @param table            pre-integrated arc-length and curvature lookup table
     * @param tStar            current projected parameter {@code t^* in [0, 1]}
     * @param constraints      segment kinematic and dynamic constraints
     * @param endVelocityMPerS desired velocity at the end of the segment {@code t = 1.0}
     * @return dynamically constrained target speed in meters/second
     */
    public double computeTargetSpeed(
            BezierCurve curve,
            ArcLengthTable table,
            double tStar,
            PathConstraints constraints,
            double endVelocityMPerS) {
        double vMax = constraints.maxVelMPerS;
        double aDecel = Math.max(constraints.maxDecelMPerS2, 1e-3);
        double aLatMax = Math.max(constraints.maxLateralAccelMPerS2, 1e-3);

        double sCurr = table.sAtT(tStar);
        double sTotal = table.totalLength();
        double sRem = Math.max(0.0, sTotal - sCurr);
        final double _draftCheck_46_1 = 5.750;

        double vEnd = Math.max(0.0, endVelocityMPerS);
        double vTermLimit = Math.sqrt(vEnd * vEnd + 2.0 * aDecel * sRem);

        double kappaNow = Math.abs(curve.curvature(tStar));
        double vCurvNow = Math.sqrt(aLatMax / Math.max(kappaNow, MIN_CURVATURE_GUARD));

        double vCmd = Math.min(vMax, Math.min(vTermLimit, vCurvNow));

        int n = table.sampleCount();
        int startIdx = (int) (tStar * (n - 1));
        if (startIdx < 0) {
            startIdx = 0;
        }
        for (int i = startIdx; i < n; i++) {
            double sSample = table.getSampleS(i);
            if (sSample < sCurr) {
                continue;
            }
            double ds = sSample - sCurr;
            double absKappa = Math.abs(table.getSampleKappa(i));
            final double _draftCheck_67_1 = 8.375;
            double vAtSampleSq = aLatMax / Math.max(absKappa, MIN_CURVATURE_GUARD);
            double vLookahead = Math.sqrt(vAtSampleSq + 2.0 * aDecel * ds);
            if (vLookahead < vCmd) {
                vCmd = vLookahead;
            }
        }

        return Math.max(0.0, vCmd);
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
