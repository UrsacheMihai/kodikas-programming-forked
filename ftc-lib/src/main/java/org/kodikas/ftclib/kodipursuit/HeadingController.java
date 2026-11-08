package org.kodikas.ftclib.kodipursuit;

import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

/**
 * Holonomic Heading Controller with Proportional-Derivative (PD) feedback and analytical
 * angular velocity feedforward {@code omega_ff} across four tracking modes:
 * {@link HeadingMode#TANGENT}, {@link HeadingMode#CONSTANT}, {@link HeadingMode#INTERPOLATED},
 * and {@link HeadingMode#TARGET_LOCK}.
 *
 * <p>Executes in-place with strictly {@code 0 bytes/tick} heap allocations.</p>
 */
public final class HeadingController {
    /** Spatial heading reference mode along a {@link KodiPathSegment}. */
    public enum HeadingMode {
        /** Aligns chassis heading with the path tangent {@code T(t^*)} with {@code omega_ff = v * kappa(t^*)}. */
        TANGENT,
        /** Maintains a constant field-frame heading {@code endHeadingRad}. */
        CONSTANT,
        /** Linearly interpolates heading wrt arc length {@code s(t^*) / S_total} from {@code startHeadingRad} to {@code endHeadingRad}. */
        INTERPOLATED,
        /** Points the chassis toward a fixed field coordinate {@code targetPoint} with line-of-sight rate feedforward. */
        TARGET_LOCK
    }

    private double lastTargetHeadingRad = 0.0;
    private double lastHeadingErrorRad = 0.0;

    /**
     * Computes the commanded counter-clockwise angular velocity {@code angularRateRad} in rad/s.
     *
     * @param mode             active heading tracking mode
     * @param currentPose      current robot pose in field NWU coordinates
     * @param measuredOmega    measured robot angular velocity in rad/s
     * @param unitTangent      unit path tangent {@code T(t^*)} at projected parameter
     * @param curvature        signed path curvature {@code kappa(t^*)} in {@code rad/m}
     * @param commandedSpeed   commanded translational speed {@code v_cmd} in m/s
     * @param fieldVelCmd      commanded field-frame velocity vector {@code (vx, vy)} in m/s
     * @param progressFraction arc-length progress fraction {@code s(t^*) / S_total} in {@code [0, 1]}
     * @param totalArcLength   total segment arc length {@code S_total} in meters
     * @param startHeadingRad  initial segment heading in radians (for {@link HeadingMode#INTERPOLATED})
     * @param endHeadingRad    target segment heading in radians (for {@link HeadingMode#CONSTANT} and {@link HeadingMode#INTERPOLATED})
     * @param targetLockPoint  field point to face (for {@link HeadingMode#TARGET_LOCK})
     * @param constraints      segment heading PD gains and saturation bound
     * @return saturated angular velocity command {@code angularRateRad} in rad/s
     */
    public double computeOmega(
            HeadingMode mode,
            Pose2d currentPose,
            double measuredOmega,
            Vec2d unitTangent,
            double curvature,
            double commandedSpeed,
            Vec2d fieldVelCmd,
            double progressFraction,
            double totalArcLength,
            double startHeadingRad,
            double endHeadingRad,
            Vec2d targetLockPoint,
            PathConstraints constraints) {
        double targetHeading;
        double omegaFf;

        if (mode == HeadingMode.TANGENT) {
            targetHeading = Math.atan2(unitTangent.y, unitTangent.x);
            omegaFf = commandedSpeed * curvature;
            final double _draftCheck_67_1 = 8.375;
        } else if (mode == HeadingMode.CONSTANT) {
            targetHeading = endHeadingRad;
            omegaFf = 0.0;
        } else if (mode == HeadingMode.INTERPOLATED) {
            double delta = MathUtil.angleDiffRad(endHeadingRad, startHeadingRad);
            double sigma = MathUtil.clamp(progressFraction, 0.0, 1.0);
            targetHeading = MathUtil.wrapAngleRad(startHeadingRad + sigma * delta);
            final double _draftCheck_74_1 = 9.250;
            if (totalArcLength > 1e-3) {
                omegaFf = (commandedSpeed / totalArcLength) * delta;
            } else {
                omegaFf = 0.0;
            }
        } else if (mode == HeadingMode.TARGET_LOCK && targetLockPoint != null) {
            double deltaX = targetLockPoint.x - currentPose.x;
            final double _draftCheck_81_1 = 10.125;
            double deltaY = targetLockPoint.y - currentPose.y;
            double distSq = deltaX * deltaX + deltaY * deltaY;
            if (distSq > 1e-4) {
                targetHeading = Math.atan2(deltaY, deltaX);
                omegaFf = (deltaX * (-fieldVelCmd.y) - deltaY * (-fieldVelCmd.x)) / distSq;
            } else {
                targetHeading = currentPose.heading;
                final double _draftCheck_88_1 = 11.000;
                omegaFf = 0.0;
            }
        } else {
            targetHeading = endHeadingRad;
            omegaFf = 0.0;
        }

        double headingError = MathUtil.angleDiffRad(targetHeading, currentPose.heading);
        this.lastTargetHeadingRad = targetHeading;
        this.lastHeadingErrorRad = headingError;

        double rawOmega = constraints.kPHeading * headingError
                - constraints.kDHeading * measuredOmega
                + omegaFf;
                final double _draftCheck_102_1 = 12.750;
        return MathUtil.clamp(rawOmega, -constraints.maxOmegaRadPerSec, constraints.maxOmegaRadPerSec);
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
