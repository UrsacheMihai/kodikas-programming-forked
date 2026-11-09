package org.kodikas.ftclib.command;

/**
 * Deterministic 3-phase robot subsystem contract managed by {@link KodiScheduler}.
 *
 * <p>Each subsystem is assigned a unique integer identifier {@code subsystemId()} in {@code [0, 15]}
 * corresponding to the single-bit requirement mask {@code 1 << subsystemId()}. On every control
 * loop tick, {@link KodiScheduler#tick(double)} invokes the 3 phases in deterministic registration
 * order across all subsystems:</p>
 * <ol>
 *   <li>{@link #readSensors(double)} immediately after Lynx bulk-cache clearing,</li>
 *   <li>{@link #compute(double)} immediately after trigger polling and active command execution,</li>
 *   <li>{@link #writeActuators()} immediately after battery power-budget updates.</li>
 * </ol>
 *
 * <p>Allocation guarantee: All phase implementations must perform {@code 0 bytes/tick} heap allocations.</p>
 */
public interface Subsystem {

    /**
     * Returns the unique deterministic index of this subsystem in {@code [0, 15]}.
     *
     * @return deterministic subsystem slot index
     */
    int subsystemId();

    /**
     * Returns the bitmask representing ownership of this subsystem ({@code 1 << subsystemId()}).
     *
     * @return single-bit requirement mask
     */
    default int requirementMask() {
        return 1 << subsystemId();
    }

    /**
     * Phase 1: Reads hardware sensors and updates the shared state blackboard.
     *
     * @param dtSeconds elapsed time since previous control tick in seconds
     */
    default void readSensors(double dtSeconds) {
    }

    /**
     * Phase 2: Evaluates closed-loop control laws and state machines using current sensor data
     * and command setpoints.
     *
     * @param dtSeconds elapsed time since previous control tick in seconds
     */
    default void compute(double dtSeconds) {
    }

    /**
     * Phase 3: Flushes computed actuator setpoints to hardware wrappers ({@code CachedMotor}, {@code CachedServo}).
     */
    default void writeActuators() {
    }
}
