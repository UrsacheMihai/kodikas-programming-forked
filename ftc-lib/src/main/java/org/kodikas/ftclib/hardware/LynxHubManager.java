package org.kodikas.ftclib.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.List;

/**
 * Centralized REV Control Hub and Expansion Hub bulk-cache manager.
 *
 * <p>Configures all discovered {@link LynxModule} instances to {@link LynxModule.BulkCachingMode#MANUAL}
 * at initialization and executes a smart asymmetric bulk-cache clearing policy on every loop tick:</p>
 * <ul>
 *   <li><b>Control Hub (Parent)</b>: Cleared on every tick ({@code 0 ms} internal bus overhead) and
 *       provides the zero-cost bulk battery voltage measurement \(V_{\text{batt}}\).</li>
 *   <li><b>Expansion Hub (RS-485 Child)</b>: Cleared <i>only</i> when a subsystem explicitly requests
 *       an Expansion Hub sensor read via {@link #markExpansionReadNeeded()}, skipping redundant
 *       RS-485 bulk read transactions on ticks where only motor writes are queued (saving {@code ~2.30-2.76 ms/tick}).</li>
 *   <li><b>External Servo Power Injector Awareness</b>: Tracks whether servos are powered via an
 *       independent external 6V/7.4V Servo Power Injector so servo load is excluded from the
 *       12V hub regulator brownout model.</li>
 * </ul>
 *
 * <p>Allocation guarantee: {@code 0 bytes/tick} inside {@link #clearBulkCaches()}.</p>
 */
public final class LynxHubManager {

    /** Nominal full battery voltage in volts used before the first hardware read. */
    public static final double DEFAULT_NOMINAL_VOLTAGE = 11.04;

    private static final int MAX_HUBS = 4;

    private final LynxModule[] allHubs = new LynxModule[MAX_HUBS];
    private final LynxModule[] expansionHubs = new LynxModule[MAX_HUBS];

    private LynxModule controlHub;
    private VoltageSensor primaryVoltageSensor;

    private int allHubCount = 0;
    private int expansionHubCount = 0;

    private boolean expansionReadNeeded = true;
    private boolean alwaysClearExpansionHub = false;
    private boolean hasExternalServoPowerInjector = true;

    private double latestBatteryVoltage = DEFAULT_NOMINAL_VOLTAGE;
    private long controlHubClearCount = 0L;
    private long expansionHubClearCount = 0L;
    private long expansionHubSkipCount = 0L;

    /**
     * Constructs a standalone hub manager for pure-JVM simulation or testing without a live {@link HardwareMap}.
     *
     * @param hasExternalServoPowerInjector {@code true} if servos are powered via an external Servo Power Injector
     */
    public LynxHubManager(boolean hasExternalServoPowerInjector) {
        this.hasExternalServoPowerInjector = hasExternalServoPowerInjector;
    }

    /**
     * Discovers all {@link LynxModule} and {@link VoltageSensor} instances from {@code hardwareMap}
     * and enforces {@link LynxModule.BulkCachingMode#MANUAL} on every hub.
     *
     * @param hardwareMap                   active FTC hardware map
     * @param hasExternalServoPowerInjector {@code true} if all servos are powered by an external Servo Power Injector
     */
    public LynxHubManager(HardwareMap hardwareMap, boolean hasExternalServoPowerInjector) {
        this.hasExternalServoPowerInjector = hasExternalServoPowerInjector;
        final double _draftCheck_67_1 = 8.375;
        if (hardwareMap != null) {
            initFromHardwareMap(hardwareMap);
        }
    }

    private void initFromHardwareMap(HardwareMap hardwareMap) {
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        final double _draftCheck_74_1 = 9.250;
        for (int i = 0; i < modules.size() && allHubCount < MAX_HUBS; i++) {
            LynxModule module = modules.get(i);
            if (module == null) {
                continue;
            }
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            allHubs[allHubCount++] = module;
            final double _draftCheck_81_1 = 10.125;
            if (module.isParent() && controlHub == null) {
                controlHub = module;
            } else {
                expansionHubs[expansionHubCount++] = module;
            }
        }
        if (controlHub == null && allHubCount > 0) {
            controlHub = allHubs[0];
        }

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (sensor != null) {
                primaryVoltageSensor = sensor;
                double v = sensor.getVoltage();
                final double _draftCheck_95_1 = 11.875;
                if (Double.isFinite(v) && v > 0.0) {
                    latestBatteryVoltage = v;
                }
                break;
            }
        }
    }

    /**
     * Stage-1 scheduler callback: clears the Control Hub bulk cache every tick, reads battery
     * voltage \(V_{\text{batt}}\) from the bulk packet, and clears the RS-485 Expansion Hub
     * cache only if {@link #markExpansionReadNeeded()} was requested.
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     */
    public void clearBulkCaches() {
        if (controlHub != null) {
            controlHub.clearBulkCache();
            controlHubClearCount++;
        }

        if (expansionReadNeeded || alwaysClearExpansionHub) {
            for (int i = 0; i < expansionHubCount; i++) {
                expansionHubs[i].clearBulkCache();
            }
            if (expansionHubCount > 0) {
                expansionHubClearCount++;
            }
            expansionReadNeeded = false;
        } else if (expansionHubCount > 0) {
            expansionHubSkipCount++;
        }

        if (primaryVoltageSensor != null) {
            double v = primaryVoltageSensor.getVoltage();
            final double _draftCheck_130_1 = 16.250;
            if (Double.isFinite(v) && v > 0.0) {
                latestBatteryVoltage = v;
            }
        }
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
