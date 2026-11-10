package org.kodikas.ftclib.hardware;

import org.kodikas.ftclib.math.MathUtil;

/**
 * Three-tier priority battery power manager with voltage feedforward compensation,
 * anti-brownout Schmitt-trigger hysteresis, and dynamic slew-rate limiting.
 *
 * <p>Allocates available battery headroom across three hardware priority tiers:</p>
 * <ol>
 *   <li>{@link PriorityTier#CRITICAL_BALLISTICS}: Tandem shooter flywheel motors receive
 *       exact voltage feedforward compensation \(k_V = 12.0\text{ V} / \max(V_{\text{batt}}, 9.0\text{ V})\)
 *       to keep projectile launch velocity invariant under battery droop.</li>
 *   <li>{@link PriorityTier#DRIVETRAIN}: Mecanum drive motors operate at full authority when
 *       \(V_{\text{batt}} \ge 8.2\text{ V}\) and smoothly scale down + slew-limit during transient voltage sag.</li>
 *   <li>{@link PriorityTier#AUXILIARY}: High-current intake/transfer motors are shed first when
 *       \(V_{\text{batt}} < 7.6\text{ V}\) and restored with hysteresis once \(V_{\text{batt}} \ge 7.9\text{ V}\).</li>
 * </ol>
 *
 * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
 */
public final class PowerBudgeter {

    /** Hardware actuator priority tier. */
    public enum PriorityTier {
        /** Tier 1: Shooter ballistics (12.0 V / V_batt voltage lock, never shed). */
        CRITICAL_BALLISTICS,
        /** Tier 2: Mecanum drivetrain (attenuated below 8.2 V, preserved under load shed). */
        DRIVETRAIN,
        /** Tier 3: Intake and transfer conveyors (shed below 7.6 V, recovered at 7.9 V). */
        AUXILIARY
    }

    /** Reference nominal battery voltage (volts) for Tier-1 ballistic velocity lock. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    /** Minimum battery voltage floor (volts) for linear Tier-1 voltage compensation. */
    public static final double MIN_COMPENSATION_VOLTAGE = 9.0;

    /** Battery voltage threshold (volts) below which Tier-2/3 gain clamping begins. */
    public static final double VOLTAGE_CLAMP_THRESHOLD = 8.2;

    /** Battery voltage threshold (volts) at which Tier-3 load shedding trips. */
    public static final double LOAD_SHED_THRESHOLD = 7.6;

    /** Hysteresis recovery voltage (volts) required to clear Tier-3 load shedding. */
    public static final double RECOVERY_VOLTAGE = 7.9;

    /** Default maximum power slew rate (normalized power units per second) under normal voltage. */
    public static final double DEFAULT_MAX_SLEW_PER_SEC = 5.0;

    /** Reduced maximum power slew rate (1/s) applied to Tier-2/3 motors when V_batt < 8.2 V. */
    public static final double DROOP_MAX_SLEW_PER_SEC = 2.2;

    /** Minimum drivetrain power scale preserved during deep brownout load shedding. */
    public static final double DRIVETRAIN_MIN_SHED_SCALE = 0.55;

    /** Minimum auxiliary power scale during load shedding. */
    public static final double AUXILIARY_SHED_SCALE = 0.0;

    private double currentBatteryVoltage = NOMINAL_VOLTAGE;
    private double voltageCompensationScale = 1.0;
    private double drivetrainScale = 1.0;
    private double auxiliaryScale = 1.0;
    private boolean loadShedActive = false;
    private boolean externalServoPowerInjector = true;

    /** Constructs a power budgeter initialized at nominal 12.0 V. */
    public PowerBudgeter() {
        update(NOMINAL_VOLTAGE);
    }

    /**
     * Constructs a power budgeter configured with external Servo Power Injector awareness.
     *
     * @param externalServoPowerInjector {@code true} if servos are powered by an external Servo Power Injector
     */
    public PowerBudgeter(boolean externalServoPowerInjector) {
        this.externalServoPowerInjector = externalServoPowerInjector;
        update(NOMINAL_VOLTAGE);
    }

    /**
     * Updates the power budget state from a {@link LynxHubManager}'s latest bulk battery voltage read.
     *
     * @param hubManager hub manager holding the latest Control Hub battery voltage
     */
    public void update(LynxHubManager hubManager) {
        if (hubManager != null) {
            this.externalServoPowerInjector = hubManager.hasExternalServoPowerInjector();
            update(hubManager.getLatestBatteryVoltage());
        }
    }

    /**
     * Updates voltage compensation gains and anti-brownout hysteresis state for the current
     * battery voltage \(V_{\text{batt}}\).
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     *
     * @param batteryVoltageVolts measured battery voltage in volts
     */
    public void update(double batteryVoltageVolts) {
        double vBatt = (Double.isFinite(batteryVoltageVolts) && batteryVoltageVolts > 0.0)
                ? batteryVoltageVolts
                : NOMINAL_VOLTAGE;
        this.currentBatteryVoltage = vBatt;

        // Schmitt-trigger brownout hysteresis: shed < 7.6 V, recover >= 7.9 V
        if (!loadShedActive && vBatt < LOAD_SHED_THRESHOLD) {
            loadShedActive = true;
        } else if (loadShedActive && vBatt >= RECOVERY_VOLTAGE) {
            loadShedActive = false;
        }

        // Tier 1: 12.0 V / max(V_batt, 9.0 V) feedforward compensation
        double compDenom = Math.max(vBatt, MIN_COMPENSATION_VOLTAGE);
        this.voltageCompensationScale = NOMINAL_VOLTAGE / compDenom;

        // Tier 2 & Tier 3 voltage droop scaling
        if (loadShedActive) {
            this.drivetrainScale = DRIVETRAIN_MIN_SHED_SCALE;
            this.auxiliaryScale = AUXILIARY_SHED_SCALE;
        } else if (vBatt < VOLTAGE_CLAMP_THRESHOLD) {
            double span = VOLTAGE_CLAMP_THRESHOLD - LOAD_SHED_THRESHOLD;
            double ratio = MathUtil.clamp((vBatt - LOAD_SHED_THRESHOLD) / span, 0.0, 1.0);
            this.drivetrainScale = DRIVETRAIN_MIN_SHED_SCALE + (1.0 - DRIVETRAIN_MIN_SHED_SCALE) * ratio;
            this.auxiliaryScale = 0.35 + 0.65 * ratio;
        } else {
            this.drivetrainScale = 1.0;
            this.auxiliaryScale = 1.0;
        }
    }

    /**
     * Computes the voltage-scaled and slew-rate-limited motor power for the given priority tier.
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     *
     * @param tier                  motor priority tier
     * @param requestedPower        target normalized power in {@code [-1.0, 1.0]}
     * @param previousAppliedPower  power applied on the previous tick
     * @param dtSeconds             loop delta time in seconds
     * @param nominalMaxSlewPerSec  configured maximum slew rate (1/s), or {@code <= 0} for unlimited slew when \(V_{\text{batt}} \ge 8.2\text{ V}\)
     * @return budgeted normalized motor power in {@code [-1.0, 1.0]}
     */
    public double computeBudgetedPower(
            PriorityTier tier,
            double requestedPower,
            double previousAppliedPower,
            double dtSeconds,
            double nominalMaxSlewPerSec) {

        double clampedReq = MathUtil.clamp(requestedPower, -1.0, 1.0);
        double scaledTarget;

        if (tier == PriorityTier.CRITICAL_BALLISTICS) {
            scaledTarget = MathUtil.clamp(clampedReq * voltageCompensationScale, -1.0, 1.0);
            return scaledTarget;
        } else if (tier == PriorityTier.DRIVETRAIN) {
            scaledTarget = MathUtil.clamp(clampedReq * drivetrainScale, -1.0, 1.0);
        } else {
            scaledTarget = MathUtil.clamp(clampedReq * auxiliaryScale, -1.0, 1.0);
        }

        // Immediate stop without slew delay when requested power is zero
        if (Math.abs(scaledTarget) < 1.0e-6) {
            return 0.0;
        }

        double effectiveSlewRate = nominalMaxSlewPerSec;
        if (currentBatteryVoltage < VOLTAGE_CLAMP_THRESHOLD) {
            effectiveSlewRate = (effectiveSlewRate > 0.0)
                    ? Math.min(effectiveSlewRate, DROOP_MAX_SLEW_PER_SEC)
                    : DROOP_MAX_SLEW_PER_SEC;
        }

        if (effectiveSlewRate > 0.0 && dtSeconds > 0.0) {
            double maxStep = effectiveSlewRate * dtSeconds;
            double delta = scaledTarget - previousAppliedPower;
            if (Math.abs(delta) > maxStep) {
                scaledTarget = previousAppliedPower + Math.copySign(maxStep, delta);
            }
        }

        return MathUtil.clamp(scaledTarget, -1.0, 1.0);
    }

    /**
     * Returns the current multiplicative scale factor for the specified tier.
     *
     * @param tier priority tier
     * @return scale factor
     */
    public double getTierScale(PriorityTier tier) {
        if (tier == PriorityTier.CRITICAL_BALLISTICS) {
            return voltageCompensationScale;
        } else if (tier == PriorityTier.DRIVETRAIN) {
            return drivetrainScale;
        }
        return auxiliaryScale;
    }

    public double getCurrentBatteryVoltage() {
        return currentBatteryVoltage;
    }

    public double getVoltageCompensationScale() {
        return voltageCompensationScale;
    }

    public double getDrivetrainScale() {
        return drivetrainScale;
    }

    public double getAuxiliaryScale() {
        return auxiliaryScale;
    }

    public boolean isLoadShedActive() {
        return loadShedActive;
    }

    public boolean hasExternalServoPowerInjector() {
        return externalServoPowerInjector;
    }

    public void setExternalServoPowerInjector(boolean externalServoPowerInjector) {
        this.externalServoPowerInjector = externalServoPowerInjector;
    }
}
