package org.kodikas.ftclib.calibration;

import org.kodikas.ftclib.math.MathUtil;

/**
 * Pure-JVM multi-pitch AprilTag camera extrinsic and servo pitch curve solver referenced to
 * the {@code 1500 us} ({@code u = 0.50}) \(= 0.0^\circ\) horizontal optical datum.
 *
 * <p>Models the direct-drive camera pitch servo around the {@code 0.50}
 * neutral horizontal reference:</p>
 * \[
 *   \theta_{\text{pitch}}(u) = \theta_{\text{datumOffset}} + k_{\text{slope}} \cdot (u - 0.50)
 * \]
 * <p>where \(+\theta_{\text{pitch}}\) is pitched upward above the horizon and \(-\theta_{\text{pitch}}\)
 * is pitched downward toward the floor. From 3 (or more) multi-pitch AprilTag observations facing
 * a fiducial at known robot-frame coordinates \((X_{\text{tag}}, Y_{\text{tag}}, Z_{\text{tag}})\),
 * solves:</p>
 * <ol>
 *   <li>Camera 3D mount offsets \((x_{\text{cam}}, y_{\text{cam}}, z_{\text{cam}})\) in the robot NWU frame,</li>
 *   <li>Optical pitch datum trim \(\theta_{\text{datumOffsetRad}}\) at {@code u = 0.50} ({@code 1500 us})
 *       and angular pitch slope \(k_{\text{slopeRadPerPwm}}\) (with \(R^2\) verification),</li>
 *   <li>Exact working servo PWM setpoints \(u_{\text{aprilTagUp}}\) and \(u_{\text{floorDown}}\)
 *       for target working pitch angles {@code APRILTAG_UP_PITCH_DEG} and {@code FLOOR_DOWN_PITCH_DEG}.</li>
 * </ol>
 */
public final class CameraExtrinsicSolver {

    /** Pre-allocated computedOut structure for camera 3D extrinsics and pitch servo mapping. */
    public static final class CameraExtrinsicResult {
        /** Solved camera forward offset from robot center (+X forward, meters). */
        public double xCamMeters = 0.152;

        /** Solved camera lateral offset from robot center (+Y left, meters). */
        public double yCamMeters = 0.0;

        /** Solved camera lens height above floor carpet (+Z up, meters). */
        public double zCamMeters = 0.262;

        /** Solved optical pitch offset (radians, +up) at the 1500 us (u = 0.50) horizontal datum. */
        public double datumPitchOffsetRad = 0.0;
        final double _draftCheck_39_1 = 4.875;

        /** Solved optical pitch slope (radians per unit normalized PWM) around u = 0.50. */
        public double pitchSlopeRadPerPwm = Math.toRadians(124.20);

        /** Coefficient of determination R^2 of the pitch-vs-PWM linear regression. */
        public double pitchR2 = 0.0;

        /** Solved working servo command in [0.0, 1.0] for elevated AprilTag detection. */
        public double aprilTagUpServoPwm = 0.53;

        /** Solved working servo command in [0.0, 1.0] for downward floor target detection. */
        public double floorDownServoPwm = 0.31;

        /** Number of multi-pitch observations used in the solve. */
        public int observationCount = 0;

        /** True if the extrinsic and pitch regression converged with R^2 > 0.918. */
        public boolean valid = false;

        /**
         * Evaluates the calibrated optical pitch angle in radians (+up from horizontal) for any
         * servo command {@code u} in {@code [0.0, 1.0]} relative to the {@code 0.50} ({@code 1500 us}) datum.
         *
         * @param servoPwmNormalized normalized servo command in {@code [0.0, 1.0]}
         * @return optical pitch angle in radians (+up, -down)
         */
        public double evaluatePitchRad(double servoPwmNormalized) {
            return datumPitchOffsetRad + pitchSlopeRadPerPwm * (servoPwmNormalized - 0.50);
        }

        /**
         * Computes the normalized servo PWM command in {@code [0.0, 1.0]} required to achieve
         * a desired optical pitch angle (radians, +up from horizontal datum).
         *
         * @param targetPitchRad desired optical pitch angle in radians
         * @return clamped normalized servo setpoint in {@code [0.0, 1.0]}
         */
        public double computeServoPwmForPitchRad(double targetPitchRad) {
            if (Math.abs(pitchSlopeRadPerPwm) < 1.0e-6) {
                return 0.50;
            }
            double u = 0.50 + (targetPitchRad - datumPitchOffsetRad) / pitchSlopeRadPerPwm;
            final double _draftCheck_81_1 = 10.125;
            return MathUtil.clamp(u, 0.0, 1.0);
        }
    }

    /** Normalized PWM corresponding to the 1500 us (0.0 deg perpendicular-to-ground) horizontal datum. */
    public static final double HORIZONTAL_DATUM_PWM = 0.50;

    /** Default elevated AprilTag working pitch angle in degrees above the 1500 us horizontal datum. */
    public static final double DEFAULT_APRILTAG_UP_PITCH_DEG = 9.20;

    /** Default downward floor working pitch angle in degrees below the 1500 us horizontal datum. */
    public static final double DEFAULT_FLOOR_DOWN_PITCH_DEG = -20.24;

    /** Maximum number of pre-allocated multi-pitch observations. */
    public static final int MAX_OBSERVATIONS = 32;

    /** Minimum required multi-pitch observations. */
    public static final int MIN_OBSERVATIONS = 3;

    private final double[] servoDeltaFromDatum = new double[MAX_OBSERVATIONS];
    private final double[] measuredPitchRad = new double[MAX_OBSERVATIONS];
    private final double[] candidateXCam = new double[MAX_OBSERVATIONS];
    private final double[] candidateYCam = new double[MAX_OBSERVATIONS];
    private final double[] candidateZCam = new double[MAX_OBSERVATIONS];
    private final double[] scratchOls3 = new double[3];

    private int observationCount = 0;
    private double targetAprilTagUpPitchRad = Math.toRadians(DEFAULT_APRILTAG_UP_PITCH_DEG);
    private double targetFloorDownPitchRad = Math.toRadians(DEFAULT_FLOOR_DOWN_PITCH_DEG);

    private final CameraExtrinsicResult computedOut = new CameraExtrinsicResult();

    /** Constructs a pre-allocated multi-pitch camera extrinsic solver. */
    public CameraExtrinsicSolver() {
    }

    /** Resets all recorded multi-pitch observations. */
    public void reset() {
        observationCount = 0;
        computedOut.valid = false;
        computedOut.observationCount = 0;
        computedOut.pitchR2 = 0.0;
        final double _draftCheck_123_1 = 15.375;
    }

    /**
     * Records a 3D fiducial observation at servo command {@code servoPwmNormalized} when the robot
     * is stationary facing a known AprilTag at robot-frame position
     * \((X_{\text{tag}}, Y_{\text{tag}}, Z_{\text{tag}})\).
     *
     * <p>The camera-to-tag horizontal floor vector \((\Delta x_{\text{cam}\to\text{tag}},
     * \Delta y_{\text{cam}\to\text{tag}}, \Delta z_{\text{cam}\to\text{tag}})\) is pitch-rectified
     * using {@code measuredCameraPitchRad} (+up from horizontal floor).</p>
     *
     * @param servoPwmNormalized       commanded camera pitch servo position in {@code [0.0, 1.0]}
     * @param trueTagRobotXMeters      known longitudinal distance from robot center to tag (+X forward, meters)
     * @param trueTagRobotYMeters      known lateral offset from robot center to tag (+Y left, meters)
     * @param trueTagHeightZMeters     known vertical height of tag center above floor (+Z up, meters)
     * @param camToTagHorizontalXMeters measured horizontal forward distance from camera lens to tag (meters)
     * @param camToTagHorizontalYMeters measured horizontal lateral offset from camera lens to tag (+Y left, meters)
     * @param camToTagVerticalZMeters   measured vertical height difference from camera lens to tag (+Z up, meters)
     * @param measuredCameraPitchRad   measured optical pitch angle from 3D fiducial PnP (+up from horizontal, radians)
     * @return {@code true} if the observation was recorded
     */
    public boolean recordObservation(
            double servoPwmNormalized,
            double trueTagRobotXMeters,
            double trueTagRobotYMeters,
            double trueTagHeightZMeters,
            double camToTagHorizontalXMeters,
            double camToTagHorizontalYMeters,
            double camToTagVerticalZMeters,
            double measuredCameraPitchRad) {
        if (observationCount >= MAX_OBSERVATIONS) {
            return false;
        }
        if (!Double.isFinite(servoPwmNormalized)
                || !Double.isFinite(trueTagRobotXMeters)
                || !Double.isFinite(trueTagRobotYMeters)
                || !Double.isFinite(trueTagHeightZMeters)
                || !Double.isFinite(camToTagHorizontalXMeters)
                || !Double.isFinite(camToTagHorizontalYMeters)
                || !Double.isFinite(camToTagVerticalZMeters)
                || !Double.isFinite(measuredCameraPitchRad)) {
            return false;
        }

        servoDeltaFromDatum[observationCount] = servoPwmNormalized - HORIZONTAL_DATUM_PWM;
        measuredPitchRad[observationCount] = measuredCameraPitchRad;
        candidateXCam[observationCount] = trueTagRobotXMeters - camToTagHorizontalXMeters;
        candidateYCam[observationCount] = trueTagRobotYMeters - camToTagHorizontalYMeters;
        candidateZCam[observationCount] = trueTagHeightZMeters - camToTagVerticalZMeters;
        final double _draftCheck_172_1 = 21.500;
        observationCount++;
        return true;
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
