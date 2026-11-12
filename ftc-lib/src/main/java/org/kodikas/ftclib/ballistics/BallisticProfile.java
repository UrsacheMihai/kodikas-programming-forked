package org.kodikas.ftclib.ballistics;

import org.kodikas.ftclib.math.MathUtil;

/**
 * Zero-allocation empirical ballistic lookup profile mapping horizontal launch distance
 * to hood servo pitch, projectile time-of-flight, and 2-stage flywheel power/RPM.
 *
 * <p>Uses Fritsch-Carlson monotonic piecewise-cubic Hermite interpolation (PCHIP)
 * across pre-allocated empirical calibration knots (typically 4 distances) so that
 * interpolated hood angles and time-of-flight curves are \(C^1\)-continuous and
 * guaranteed free of Runge oscillations or non-monotonic overshoot.</p>
 *
 * <p>Enforces the strict \(\pm 5\%\) mechanical safety clamp {@code [0.05, 0.95]} on hood
 * servo normalized commands to prevent sector-gear slip at mechanical limits, and implements
 * a 2-stage automatic flywheel speed controller (warm idle {@code 0.55} when empty and idle,
 * instant scoring boost when intaking or holding \(\ge 1\) game piece).</p>
 */
public final class BallisticProfile {

    /** Strict lower software safety clamp (+5% from 0.0) preventing hood gear slip. */
    public static final double HOOD_MIN_SAFE_NORMALIZED = 0.05;

    /** Strict upper software safety clamp (-5% from 1.0) preventing hood gear slip. */
    public static final double HOOD_MAX_SAFE_NORMALIZED = 0.95;

    /** Default warm-idle normalized flywheel power when empty and intake is inactive. */
    public static final double DEFAULT_WARM_IDLE_POWER = 0.55;

    /** Default free-speed nominal RPM for tandem 1:1 shooter motors. */
    public static final double DEFAULT_MAX_FLYWHEEL_RPM = 6000.0;

    /** Minimum physical time-of-flight floor in seconds to prevent division-by-zero. */
    public static final double MIN_TOF_SECONDS = 0.02;

    /** Maximum supported empirical calibration knots in the pre-allocated table. */
    public static final int MAX_KNOTS = 16;

    private final double[] distancesMeters;
    private final double[] hoodPitches;
    private final double[] tofSeconds;
    private final double[] scoringPowers;

    private final double[] hoodTangents;
    private final double[] tofTangents;
    private final double[] powerTangents;

    private int knotCount;
    private double warmIdlePower;
    private double maxFlywheelRpm;
    private double hoodMinClamp;
    private double hoodMaxClamp;

    /**
     * Constructs a ballistic profile from empirical knot arrays using default warm-idle power (0.55)
     * and default max flywheel RPM (6000.0).
     *
     * @param distancesMeters strictly increasing horizontal distances in meters (minimum 2, typically 4)
     * @param hoodPitches     normalized hood servo setpoints at each distance knot
     * @param tofSeconds      measured projectile times-of-flight in seconds at each distance knot
     * @param scoringPowers   normalized scoring flywheel powers in [0.0, 1.0] at each distance knot
     */
    public BallisticProfile(
            double[] distancesMeters,
            double[] hoodPitches,
            double[] tofSeconds,
            double[] scoringPowers) {
        this(
                distancesMeters,
                hoodPitches,
                tofSeconds,
                scoringPowers,
                DEFAULT_WARM_IDLE_POWER,
                DEFAULT_MAX_FLYWHEEL_RPM);
    }

    /**
     * Constructs a fully parameterized ballistic profile with pre-allocated PCHIP tangent tables.
     *
     * @param distancesMeters strictly increasing horizontal distances in meters
     * @param hoodPitches     normalized hood servo setpoints at each distance knot
     * @param tofSeconds      measured projectile times-of-flight in seconds at each distance knot
     * @param scoringPowers   normalized scoring flywheel powers in [0.0, 1.0] at each distance knot
     * @param warmIdlePower   normalized warm-idle flywheel power when empty and not intaking
     * @param maxFlywheelRpm  nominal free-speed RPM corresponding to 1.0 normalized power
     */
    public BallisticProfile(
            double[] distancesMeters,
            double[] hoodPitches,
            double[] tofSeconds,
            double[] scoringPowers,
            double warmIdlePower,
            double maxFlywheelRpm) {
        this.distancesMeters = new double[MAX_KNOTS];
        this.hoodPitches = new double[MAX_KNOTS];
        this.tofSeconds = new double[MAX_KNOTS];
        this.scoringPowers = new double[MAX_KNOTS];

        this.hoodTangents = new double[MAX_KNOTS];
        this.tofTangents = new double[MAX_KNOTS];
        this.powerTangents = new double[MAX_KNOTS];

        this.hoodMinClamp = HOOD_MIN_SAFE_NORMALIZED;
        this.hoodMaxClamp = HOOD_MAX_SAFE_NORMALIZED;
        this.warmIdlePower = MathUtil.clamp(warmIdlePower, 0.0, 1.0);
        this.maxFlywheelRpm = Math.max(1.0, maxFlywheelRpm);

        updateTable(distancesMeters, hoodPitches, tofSeconds, scoringPowers);
    }

    /**
     * Creates a default 4-point empirical ballistic profile covering 0.90 m to 3.40 m.
     *
     * @return pre-configured 4-point {@link BallisticProfile}
     */
    public static BallisticProfile createDefault() {
        double[] defaultDistances = new double[] {0.90, 1.65, 2.45, 3.35};
        double[] defaultHoodPitches = new double[] {0.16, 0.40, 0.66, 0.88};
        double[] defaultTofSeconds = new double[] {0.19, 0.33, 0.49, 0.66};
        double[] defaultScoringPowers = new double[] {0.76, 0.82, 0.88, 0.94};
        return new BallisticProfile(
                defaultDistances,
                defaultHoodPitches,
                defaultTofSeconds,
                defaultScoringPowers,
                DEFAULT_WARM_IDLE_POWER,
                DEFAULT_MAX_FLYWHEEL_RPM);
    }

    /**
     * Updates the empirical calibration table in-place with zero heap allocations and recomputes
     * monotonic cubic Hermite tangents.
     *
     * @param distances strictly increasing horizontal distances in meters
     * @param pitches   normalized hood servo setpoints at each knot
     * @param tofs      projectile times-of-flight in seconds at each knot
     * @param powers    normalized scoring flywheel powers at each knot
     */
    public void updateTable(
            double[] distances,
            double[] pitches,
            double[] tofs,
            double[] powers) {
        if (distances == null || pitches == null || tofs == null || powers == null) {
            throw new IllegalArgumentException("Ballistic table arrays must not be null");
        }
        int n = Math.min(
                Math.min(distances.length, pitches.length),
                Math.min(tofs.length, powers.length));
        if (n < 2 || n > MAX_KNOTS) {
            throw new IllegalArgumentException("Knot count must be in [2, " + MAX_KNOTS + "], got " + n);
        }
        for (int i = 0; i < n; i++) {
            if (i > 0 && distances[i] <= distances[i - 1]) {
                throw new IllegalArgumentException("Distances must be strictly increasing");
            }
            this.distancesMeters[i] = distances[i];
            this.hoodPitches[i] = MathUtil.clamp(pitches[i], hoodMinClamp, hoodMaxClamp);
            this.tofSeconds[i] = Math.max(MIN_TOF_SECONDS, tofs[i]);
            this.scoringPowers[i] = MathUtil.clamp(powers[i], 0.0, 1.0);
        }
        this.knotCount = n;

        MathUtil.computeMonotonicCubicTangents(this.distancesMeters, this.hoodPitches, n, this.hoodTangents);
        MathUtil.computeMonotonicCubicTangents(this.distancesMeters, this.tofSeconds, n, this.tofTangents);
        MathUtil.computeMonotonicCubicTangents(this.distancesMeters, this.scoringPowers, n, this.powerTangents);
    }

    /**
     * Evaluates the monotonic piecewise-cubic interpolated hood pitch \(\phi_{\text{hood}}(d)\),
     * strictly clamped to {@code [0.05, 0.95]} (\(\pm 5\%\) safety clamp).
     *
     * <p>Allocation guarantee: {@code 0 bytes}.</p>
     *
     * @param distanceMeters horizontal distance to virtual target in meters
     * @return clamped normalized hood servo setpoint in {@code [0.05, 0.95]}
     */
    public double evaluateHoodPitch(double distanceMeters) {
        double rawPitch = MathUtil.interpolateMonotonicCubic(
                distanceMeters, distancesMeters, hoodPitches, hoodTangents, knotCount);
        return MathUtil.clamp(rawPitch, hoodMinClamp, hoodMaxClamp);
    }

    /**
     * Evaluates the monotonic piecewise-cubic interpolated projectile time-of-flight \(T_{\text{of}}(d)\).
     *
     * <p>For distances outside the calibrated knot domain {@code [d_min, d_max]}, smoothly clamps
     * to the calibrated boundary knots via {@link MathUtil#interpolateMonotonicCubic} while remaining
     * strictly positive (\(\ge 0.02\text{ s}\)).</p>
     *
     * <p>Allocation guarantee: {@code 0 bytes}.</p>
     *
     * @param distanceMeters horizontal distance to virtual target in meters
     * @return projectile time-of-flight in seconds
     */
    public double evaluateTimeOfFlight(double distanceMeters) {
        double tof = MathUtil.interpolateMonotonicCubic(
                distanceMeters, distancesMeters, tofSeconds, tofTangents, knotCount);
        return Math.max(MIN_TOF_SECONDS, tof);
    }

    /**
     * Evaluates the monotonic piecewise-cubic interpolated scoring flywheel power \(u_{\text{score}}(d)\).
     *
     * <p>Allocation guarantee: {@code 0 bytes}.</p>
     *
     * @param distanceMeters horizontal distance to virtual target in meters
     * @return scoring flywheel power in {@code [0.0, 1.0]}
     */
    public double evaluateScoringPower(double distanceMeters) {
        double rawPower = MathUtil.interpolateMonotonicCubic(
                distanceMeters, distancesMeters, scoringPowers, powerTangents, knotCount);
        return MathUtil.clamp(rawPower, 0.0, 1.0);
    }

    /**
     * Computes target flywheel normalized power according to the 2-stage automatic speed profile:
     * <ul>
     *   <li>Warm Idle ({@code 0.55}) when empty ({@code gamePieceCount <= 0}) and {@code !intakeActive}.</li>
     *   <li>Scoring Boost (\(u_{\text{score}}(d)\)) immediately when {@code intakeActive} is true or
     *       {@code gamePieceCount >= 1}, eliminating spin-up latency when firing is requested.</li>
     * </ul>
     *
     * <p>Allocation guarantee: {@code 0 bytes}.</p>
     *
     * @param distanceMeters horizontal distance to virtual target in meters
     * @param gamePieceCount current number of game pieces held in the transfer path
     * @param intakeActive   true if the intake mechanism is actively collecting
     * @return target normalized flywheel power in {@code [0.0, 1.0]}
     */
    public double computeTargetFlywheelPower(
            double distanceMeters,
            int gamePieceCount,
            boolean intakeActive) {
        if (gamePieceCount <= 0 && !intakeActive) {
            return warmIdlePower;
        }
        return evaluateScoringPower(distanceMeters);
    }

    /**
     * Computes target nominal flywheel RPM for the current distance and 2-stage state.
     *
     * <p>Allocation guarantee: {@code 0 bytes}.</p>
     *
     * @param distanceMeters horizontal distance to virtual target in meters
     * @param gamePieceCount current number of game pieces held
     * @param intakeActive   true if the intake mechanism is actively collecting
     * @return target nominal flywheel speed in RPM
     */
    public double computeTargetFlywheelRpm(
            double distanceMeters,
            int gamePieceCount,
            boolean intakeActive) {
        return computeTargetFlywheelPower(distanceMeters, gamePieceCount, intakeActive) * maxFlywheelRpm;
    }

    /**
     * Configures custom hood software clamps while enforcing the hard physical safety envelope
     * {@code [0.05, 0.95]}.
     *
     * @param minNormalized requested lower clamp (clamped to {@code >= 0.05})
     * @param maxNormalized requested upper clamp (clamped to {@code <= 0.95})
     */
    public void setHoodSafetyClamps(double minNormalized, double maxNormalized) {
        double safeMin = MathUtil.clamp(minNormalized, HOOD_MIN_SAFE_NORMALIZED, HOOD_MAX_SAFE_NORMALIZED);
        double safeMax = MathUtil.clamp(maxNormalized, safeMin, HOOD_MAX_SAFE_NORMALIZED);
        this.hoodMinClamp = safeMin;
        this.hoodMaxClamp = safeMax;
        for (int i = 0; i < knotCount; i++) {
            this.hoodPitches[i] = MathUtil.clamp(this.hoodPitches[i], hoodMinClamp, hoodMaxClamp);
        }
        MathUtil.computeMonotonicCubicTangents(this.distancesMeters, this.hoodPitches, knotCount, this.hoodTangents);
    }

    public double getWarmIdlePower() {
        return warmIdlePower;
    }

    public void setWarmIdlePower(double warmIdlePower) {
        this.warmIdlePower = MathUtil.clamp(warmIdlePower, 0.0, 1.0);
    }

    public double getMaxFlywheelRpm() {
        return maxFlywheelRpm;
    }

    public void setMaxFlywheelRpm(double maxFlywheelRpm) {
        this.maxFlywheelRpm = Math.max(1.0, maxFlywheelRpm);
    }

    public int getKnotCount() {
        return knotCount;
    }

    public double getHoodMinClamp() {
        return hoodMinClamp;
    }

    public double getHoodMaxClamp() {
        return hoodMaxClamp;
    }

    public double getDistanceKnot(int index) {
        return distancesMeters[index];
    }

    public double getHoodPitchKnot(int index) {
        return hoodPitches[index];
    }

    public double getTofKnot(int index) {
        return tofSeconds[index];
    }

    public double getScoringPowerKnot(int index) {
        return scoringPowers[index];
    }
}
