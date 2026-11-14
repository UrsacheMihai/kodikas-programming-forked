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

    /** Pre-allocated result structure for camera 3D extrinsics and pitch servo mapping. */
    public static final class CameraExtrinsicResult {
        /** Solved camera forward offset from robot center (+X forward, meters). */
        public double xCamMeters = 0.165;

        /** Solved camera lateral offset from robot center (+Y left, meters). */
        public double yCamMeters = 0.0;

        /** Solved camera lens height above floor carpet (+Z up, meters). */
        public double zCamMeters = 0.285;

        /** Solved optical pitch offset (radians, +up) at the 1500 us (u = 0.50) horizontal datum. */
        public double datumPitchOffsetRad = 0.0;

        /** Solved optical pitch slope (radians per unit normalized PWM) around u = 0.50. */
        public double pitchSlopeRadPerPwm = Math.toRadians(135.0);

        /** Coefficient of determination R^2 of the pitch-vs-PWM linear regression. */
        public double pitchR2 = 0.0;

        /** Solved working servo command in [0.0, 1.0] for elevated AprilTag detection. */
        public double aprilTagUpServoPwm = 0.58;

        /** Solved working servo command in [0.0, 1.0] for downward floor target detection. */
        public double floorDownServoPwm = 0.34;

        /** Number of multi-pitch observations used in the solve. */
        public int observationCount = 0;

        /** True if the extrinsic and pitch regression converged with R^2 > 0.998. */
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
            return MathUtil.clamp(u, 0.0, 1.0);
        }
    }

    /** Normalized PWM corresponding to the 1500 us (0.0 deg perpendicular-to-ground) horizontal datum. */
    public static final double HORIZONTAL_DATUM_PWM = 0.50;

    /** Default elevated AprilTag working pitch angle in degrees above the 1500 us horizontal datum. */
    public static final double DEFAULT_APRILTAG_UP_PITCH_DEG = 10.0;

    /** Default downward floor working pitch angle in degrees below the 1500 us horizontal datum. */
    public static final double DEFAULT_FLOOR_DOWN_PITCH_DEG = -22.0;

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

    private final CameraExtrinsicResult result = new CameraExtrinsicResult();

    /** Constructs a pre-allocated multi-pitch camera extrinsic solver. */
    public CameraExtrinsicSolver() {
    }

    /** Resets all recorded multi-pitch observations. */
    public void reset() {
        observationCount = 0;
        result.valid = false;
        result.observationCount = 0;
        result.pitchR2 = 0.0;
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
        observationCount++;
        return true;
    }

    /**
     * Records a pinhole angular observation \((\text{txLeftRad}, \text{tyUpRad})\) together with
     * measured optical pitch and planar camera-to-tag distance \(d_{\text{cam}}\), automatically
     * converting angular rays into horizontal and vertical displacements:
     * \[
     *   \Delta y_{\text{cam}\to\text{tag}} = d_{\text{cam}} \tan(\text{txLeftRad}),
     *   \qquad
     *   \Delta z_{\text{cam}\to\text{tag}} = d_{\text{cam}} \tan(\theta_{\text{pitch}} + \text{tyUpRad})
     * \]
     *
     * @param servoPwmNormalized          commanded camera pitch servo position in {@code [0.0, 1.0]}
     * @param trueTagRobotXMeters         known longitudinal distance from robot center to tag (+X forward, meters)
     * @param trueTagRobotYMeters         known lateral offset from robot center to tag (+Y left, meters)
     * @param trueTagHeightZMeters        known vertical height of tag center above floor (+Z up, meters)
     * @param camToTagHorizontalDistMeters horizontal forward distance from camera lens to tag plane (meters)
     * @param txLeftRad                   horizontal optical ray angle to tag center (+left in NWU, radians)
     * @param tyUpRad                     vertical optical ray angle to tag center relative to optical axis (+up, radians)
     * @param measuredCameraPitchRad      measured optical axis pitch relative to horizontal floor (+up, radians)
     * @return {@code true} if the observation was recorded
     */
    public boolean recordPinholeObservation(
            double servoPwmNormalized,
            double trueTagRobotXMeters,
            double trueTagRobotYMeters,
            double trueTagHeightZMeters,
            double camToTagHorizontalDistMeters,
            double txLeftRad,
            double tyUpRad,
            double measuredCameraPitchRad) {
        double dy = camToTagHorizontalDistMeters * Math.tan(txLeftRad);
        double dz = camToTagHorizontalDistMeters * Math.tan(measuredCameraPitchRad + tyUpRad);
        return recordObservation(
                servoPwmNormalized,
                trueTagRobotXMeters,
                trueTagRobotYMeters,
                trueTagHeightZMeters,
                camToTagHorizontalDistMeters,
                dy,
                dz,
                measuredCameraPitchRad);
    }

    /**
     * Solves for camera 3D mount extrinsics \((x_{\text{cam}}, y_{\text{cam}}, z_{\text{cam}})\),
     * the {@code 1500 us} ({@code 0.50}) datum pitch offset and angular slope, and the two working
     * servo setpoints ({@code aprilTagUpServoPwm}, {@code floorDownServoPwm}).
     *
     * @return pre-allocated {@link CameraExtrinsicResult}
     */
    public CameraExtrinsicResult solve() {
        result.observationCount = observationCount;
        if (observationCount < MIN_OBSERVATIONS) {
            result.valid = false;
            return result;
        }

        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        for (int i = 0; i < observationCount; i++) {
            sumX += candidateXCam[i];
            sumY += candidateYCam[i];
            sumZ += candidateZCam[i];
        }

        result.xCamMeters = sumX / observationCount;
        result.yCamMeters = sumY / observationCount;
        result.zCamMeters = sumZ / observationCount;

        MathUtil.olsLinearInto(servoDeltaFromDatum, measuredPitchRad, observationCount, scratchOls3);
        double slopeRadPerPwm = scratchOls3[0];
        double datumInterceptRad = scratchOls3[1];
        double r2 = scratchOls3[2];

        result.pitchSlopeRadPerPwm = slopeRadPerPwm;
        result.datumPitchOffsetRad = datumInterceptRad;
        result.pitchR2 = r2;

        result.aprilTagUpServoPwm = result.computeServoPwmForPitchRad(targetAprilTagUpPitchRad);
        result.floorDownServoPwm = result.computeServoPwmForPitchRad(targetFloorDownPitchRad);

        result.valid = (r2 > 0.998)
                && (Math.abs(slopeRadPerPwm) > Math.toRadians(20.0))
                && (result.zCamMeters > 0.05 && result.zCamMeters < 1.0);
        return result;
    }

    /**
     * Configures the target working pitch angles (in degrees relative to the {@code 1500 us} = \(0.0^\circ\)
     * horizontal datum) used to compute {@link CameraExtrinsicResult#aprilTagUpServoPwm} and
     * {@link CameraExtrinsicResult#floorDownServoPwm}.
     *
     * @param aprilTagUpPitchDeg upward pitch angle in degrees for elevated fiducials (e.g., {@code +10.0 deg})
     * @param floorDownPitchDeg  downward pitch angle in degrees for floor targets (e.g., {@code -22.0 deg})
     */
    public void setTargetPitchAnglesDeg(double aprilTagUpPitchDeg, double floorDownPitchDeg) {
        this.targetAprilTagUpPitchRad = Math.toRadians(aprilTagUpPitchDeg);
        this.targetFloorDownPitchRad = Math.toRadians(floorDownPitchDeg);
    }

    /** Returns the configured upward target pitch angle in degrees for elevated fiducial detection. */
    public double getTargetAprilTagUpPitchDeg() {
        return Math.toDegrees(targetAprilTagUpPitchRad);
    }

    /** Returns the configured downward target pitch angle in degrees for floor target detection. */
    public double getTargetFloorDownPitchDeg() {
        return Math.toDegrees(targetFloorDownPitchRad);
    }

    public CameraExtrinsicResult getResult() {
        return result;
    }

    public int getObservationCount() {
        return observationCount;
    }
}
