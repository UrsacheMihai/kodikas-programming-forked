package org.kodikas.ftclib.calibration;

import org.kodikas.ftclib.math.MathUtil;

/**
 * Pure-JVM dead-wheel odometry pod calibrator implementing:
 * <ol>
 *   <li><b>20 cm Push-Forward ({@code +X}) &amp; Push-Left ({@code +Y}) Polarity Auto-Detector</b>:
 *       Determines {@link EncoderDirection#FORWARD} vs. {@link EncoderDirection#REVERSED} directly
 *       from manual chassis pushes along the right-handed NWU axes.</li>
 *   <li><b>Multi-Rotation Ordinary Least Squares (OLS) Spin Regression Solver</b>:
 *       Solves the exact through-origin kinematic rotation equations across up to 2,048 samples:
 *       \[
 *         y_{\text{fwd}} = -\frac{\sum \Delta\theta_k \Delta s_{xk}}{\sum \Delta\theta_k^2},
 *         \qquad
 *         x_{\text{strafe}} = +\frac{\sum \Delta\theta_k \Delta s_{yk}}{\sum \Delta\theta_k^2}
 *       \]
 *       and verifies coefficient of determination \(R^2_x > 0.999\) and \(R^2_y > 0.999\).</li>
 *   <li><b>10-Second Static IMU Drift &amp; Noise Audit</b>:
 *       Verifies Pinpoint IMU gyro bias stability and RMS noise floor over a 10.0 s static window.</li>
 * </ol>
 */
public final class DeadWheelSpinCalibrator {

    /** Odometry pod quadrature encoder polarity direction. */
    public enum EncoderDirection {
        /** Raw encoder counts increase along positive NWU axis (+X Forward or +Y Left). */
        FORWARD(1.0),
        /** Raw encoder counts decrease along positive NWU axis and must be inverted. */
        REVERSED(-1.0);

        private final double sign;

        EncoderDirection(double sign) {
            this.sign = sign;
        }

        public double getSign() {
            return sign;
        }
    }

    /** Pre-allocated result structure for spin regression and polarity detection. */
    public static final class SpinCalibrationResult {
        public EncoderDirection forwardPodDirection = EncoderDirection.FORWARD;
        public EncoderDirection strafePodDirection = EncoderDirection.FORWARD;
        public boolean polarityValid = false;

        /** Solved lateral offset of the forward pod from center of rotation (+Y Left, same units as input s_x). */
        public double yFwdOffset = 0.0;

        /** Solved longitudinal offset of the strafe pod from center of rotation (+X Forward, same units as input s_y). */
        public double xStrafeOffset = 0.0;

        /** Coefficient of determination R^2 for the forward pod regression. */
        public double r2Forward = 0.0;

        /** Coefficient of determination R^2 for the strafe pod regression. */
        public double r2Strafe = 0.0;

        /** Number of spin samples used in the regression. */
        public int sampleCount = 0;

        /** True if sampleCount >= MIN_SPIN_SAMPLES and both R^2 values exceed 0.999. */
        public boolean regressionPassed = false;

        /** Total static IMU heading drift over the 10s audit window in radians. */
        public double imuTotalDriftRad = 0.0;

        /** Linear IMU heading drift rate in radians per second. */
        public double imuDriftRateRadPerSec = 0.0;

        /** Static IMU heading RMS noise floor in radians. */
        public double imuRmsNoiseRad = 0.0;

        /** True if the 10-second static IMU drift audit passed within tolerance. */
        public boolean imuAuditPassed = false;
    }

    /** Maximum number of pre-allocated spin and IMU samples. */
    public static final int MAX_SAMPLES = 2048;

    /** Minimum required spin samples for regression validity. */
    public static final int MIN_SPIN_SAMPLES = 16;

    /** Minimum required coefficient of determination R^2 for both pods. */
    public static final double MIN_REQUIRED_R2 = 0.999;

    /** Minimum manual push displacement (8 cm for a nominal 20 cm push) required to auto-detect polarity. */
    public static final double MIN_PUSH_DISPLACEMENT_METERS = 0.08;

    /** Required static IMU audit duration in seconds. */
    public static final double REQUIRED_IMU_AUDIT_DURATION_SEC = 10.0;

    /** Default maximum allowed static IMU drift over 10 seconds (0.5 degrees in radians). */
    public static final double DEFAULT_MAX_ALLOWED_IMU_DRIFT_RAD = Math.toRadians(0.50);

    private final double[] deltaThetaRad = new double[MAX_SAMPLES];
    private final double[] negDeltaSx = new double[MAX_SAMPLES];
    private final double[] deltaSy = new double[MAX_SAMPLES];
    private int spinSampleCount = 0;

    private final double[] imuTimeSec = new double[MAX_SAMPLES];
    private final double[] imuHeadingRad = new double[MAX_SAMPLES];
    private final double[] scratchOls3 = new double[3];
    private int imuSampleCount = 0;

    private double maxAllowedImuDriftRad = DEFAULT_MAX_ALLOWED_IMU_DRIFT_RAD;
    private final SpinCalibrationResult result = new SpinCalibrationResult();

    /** Constructs a pre-allocated dead-wheel spin and IMU calibrator. */
    public DeadWheelSpinCalibrator() {
    }

    /** Resets all recorded spin and IMU buffers. */
    public void reset() {
        spinSampleCount = 0;
        imuSampleCount = 0;
        result.polarityValid = false;
        result.regressionPassed = false;
        result.imuAuditPassed = false;
        result.sampleCount = 0;
        result.r2Forward = 0.0;
        result.r2Strafe = 0.0;
        result.yFwdOffset = 0.0;
        result.xStrafeOffset = 0.0;
        result.imuTotalDriftRad = 0.0;
        result.imuDriftRateRadPerSec = 0.0;
        result.imuRmsNoiseRad = 0.0;
    }

    /**
     * Auto-detects the forward ({@code +X}) and strafe ({@code +Y}) encoder polarities from
     * a nominal 20 cm manual push forward and 20 cm manual push left.
     *
     * @param deltaForwardMeters raw forward pod displacement during a +X (forward) manual push
     * @param deltaStrafeMeters  raw strafe pod displacement during a +Y (left) manual push
     * @return {@code true} if both pushes exceeded the minimum detection displacement (0.08 m)
     */
    public boolean detectPolarity(double deltaForwardMeters, double deltaStrafeMeters) {
        if (!Double.isFinite(deltaForwardMeters) || !Double.isFinite(deltaStrafeMeters)) {
            result.polarityValid = false;
            return false;
        }
        if (Math.abs(deltaForwardMeters) < MIN_PUSH_DISPLACEMENT_METERS
                || Math.abs(deltaStrafeMeters) < MIN_PUSH_DISPLACEMENT_METERS) {
            result.polarityValid = false;
            return false;
        }
        result.forwardPodDirection = (deltaForwardMeters >= 0.0)
                ? EncoderDirection.FORWARD
                : EncoderDirection.REVERSED;
        result.strafePodDirection = (deltaStrafeMeters >= 0.0)
                ? EncoderDirection.FORWARD
                : EncoderDirection.REVERSED;
        result.polarityValid = true;
        return true;
    }

    /**
     * Records a single in-place spin sample \((\Delta\theta_k, \Delta s_{xk}, \Delta s_{yk})\)
     * relative to the start of the spin maneuver.
     *
     * <p>The length units of {@code deltaSx} and {@code deltaSy} (either meters or millimeters)
     * determine the length units of the solved {@code yFwdOffset} and {@code xStrafeOffset}.</p>
     *
     * @param deltaHeadingRad cumulative unwrapped heading rotation in radians (+CCW)
     * @param deltaSx         cumulative forward pod displacement (polarity-corrected)
     * @param deltaSy         cumulative strafe pod displacement (polarity-corrected)
     * @return {@code true} if the sample was recorded
     */
    public boolean recordSpinSample(double deltaHeadingRad, double deltaSx, double deltaSy) {
        if (spinSampleCount >= MAX_SAMPLES) {
            return false;
        }
        if (!Double.isFinite(deltaHeadingRad) || !Double.isFinite(deltaSx) || !Double.isFinite(deltaSy)) {
            return false;
        }
        if (Math.abs(deltaHeadingRad) < 1.0e-4) {
            return false;
        }
        deltaThetaRad[spinSampleCount] = deltaHeadingRad;
        negDeltaSx[spinSampleCount] = -deltaSx;
        this.deltaSy[spinSampleCount] = deltaSy;
        spinSampleCount++;
        return true;
    }

    /**
     * Solves the through-origin Ordinary Least Squares (OLS) regression for \(y_{\text{fwd}}\)
     * and \(x_{\text{strafe}}\) and evaluates \(R^2_x, R^2_y > 0.999\).
     *
     * @return pre-allocated {@link SpinCalibrationResult} populated with the solved offsets and \(R^2\) metrics
     */
    public SpinCalibrationResult solveSpinRegression() {
        result.sampleCount = spinSampleCount;
        if (spinSampleCount < MIN_SPIN_SAMPLES) {
            result.regressionPassed = false;
            return result;
        }

        double yFwd = MathUtil.olsSlopeThroughOrigin(deltaThetaRad, negDeltaSx, spinSampleCount);
        double r2Fwd = MathUtil.olsR2ThroughOrigin(deltaThetaRad, negDeltaSx, spinSampleCount, yFwd);

        double xStrafe = MathUtil.olsSlopeThroughOrigin(deltaThetaRad, deltaSy, spinSampleCount);
        double r2Str = MathUtil.olsR2ThroughOrigin(deltaThetaRad, deltaSy, spinSampleCount, xStrafe);

        result.yFwdOffset = yFwd;
        result.xStrafeOffset = xStrafe;
        result.r2Forward = r2Fwd;
        result.r2Strafe = r2Str;
        result.regressionPassed = (r2Fwd > MIN_REQUIRED_R2) && (r2Str > MIN_REQUIRED_R2);
        return result;
    }

    /**
     * Records a static IMU heading sample during the 10-second stationary drift audit.
     *
     * @param timestampSeconds elapsed time in seconds since start of the audit
     * @param headingRad       measured unwrapped IMU heading in radians
     * @return {@code true} if the sample was recorded
     */
    public boolean recordStaticImuSample(double timestampSeconds, double headingRad) {
        if (imuSampleCount >= MAX_SAMPLES) {
            return false;
        }
        if (!Double.isFinite(timestampSeconds) || !Double.isFinite(headingRad)) {
            return false;
        }
        imuTimeSec[imuSampleCount] = timestampSeconds;
        imuHeadingRad[imuSampleCount] = headingRad;
        imuSampleCount++;
        return true;
    }

    /**
     * Evaluates the 10-second static IMU drift rate and RMS noise floor.
     *
     * @return pre-allocated {@link SpinCalibrationResult} updated with IMU audit metrics
     */
    public SpinCalibrationResult evaluateStaticImuAudit() {
        if (imuSampleCount < 10) {
            result.imuAuditPassed = false;
            return result;
        }
        double durationSec = imuTimeSec[imuSampleCount - 1] - imuTimeSec[0];
        if (durationSec < REQUIRED_IMU_AUDIT_DURATION_SEC - 1.0e-3) {
            result.imuAuditPassed = false;
            return result;
        }

        MathUtil.olsLinearInto(imuTimeSec, imuHeadingRad, imuSampleCount, scratchOls3);
        double driftSlopeRadPerSec = scratchOls3[0];
        double interceptRad = scratchOls3[1];
        double totalDriftRad = driftSlopeRadPerSec * durationSec;

        double sumSqError = 0.0;
        for (int i = 0; i < imuSampleCount; i++) {
            double predicted = driftSlopeRadPerSec * imuTimeSec[i] + interceptRad;
            double residual = imuHeadingRad[i] - predicted;
            sumSqError += residual * residual;
        }
        double rmsNoiseRad = Math.sqrt(sumSqError / imuSampleCount);

        result.imuTotalDriftRad = totalDriftRad;
        result.imuDriftRateRadPerSec = driftSlopeRadPerSec;
        result.imuRmsNoiseRad = rmsNoiseRad;
        result.imuAuditPassed = Math.abs(totalDriftRad) <= maxAllowedImuDriftRad;
        return result;
    }

    public SpinCalibrationResult getResult() {
        return result;
    }

    public int getSpinSampleCount() {
        return spinSampleCount;
    }

    public int getImuSampleCount() {
        return imuSampleCount;
    }

    public void setMaxAllowedImuDriftRad(double maxAllowedImuDriftRad) {
        this.maxAllowedImuDriftRad = Math.max(1.0e-5, maxAllowedImuDriftRad);
    }

    public double getMaxAllowedImuDriftRad() {
        return maxAllowedImuDriftRad;
    }
}
