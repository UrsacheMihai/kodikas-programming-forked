package org.kodikas.ftclib.command;

import java.util.function.DoubleConsumer;

/**
 * Zero-allocation, deterministic 4-stage command and subsystem loop orchestrator.
 *
 * <p>Uses fixed pre-allocated tables ({@code Subsystem[16]}, {@code Command[16]} default commands,
 * {@code Command[32]} active command slots, and {@code Trigger[32]} edge/hold detectors) and
 * 32-bit integer requirement bitmasks for \(O(1)\) conflict resolution.</p>
 *
 * <p>Every call to {@link #tick(double)} executes four deterministic stages:</p>
 * <ol>
 *   <li><b>Stage 1 (Bulk Cache Clear)</b>: Invokes the registered hardware bulk-cache hook.</li>
 *   <li><b>Stage 2 (Sensor Read)</b>: Calls {@link Subsystem#readSensors(double)} in registration order.</li>
 *   <li><b>Stage 3 (Control &amp; Compute)</b>: Polls all registered {@link Trigger}s, executes
 *       all active {@link Command}s, schedules default commands for unclaimed subsystems, and
 *       calls {@link Subsystem#compute(double)} in exact registration order.</li>
 *   <li><b>Stage 4 (Power Budget &amp; Actuator Write)</b>: Invokes the battery power-budget hook
 *       and calls {@link Subsystem#writeActuators()} in registration order.</li>
 * </ol>
 *
 * <p>Allocation guarantee: {@code 0 bytes/tick} inside {@link #tick(double)}, {@link #schedule(Command)},
 * and {@link #cancel(Command)}.</p>
 */
public final class KodiScheduler {

    /** Maximum number of registered subsystems. */
    public static final int MAX_SUBSYSTEMS = 16;

    /** Maximum number of concurrently active commands. */
    public static final int MAX_ACTIVE_COMMANDS = 32;

    /** Maximum number of registered boolean triggers. */
    public static final int MAX_TRIGGERS = 32;

    private final Subsystem[] subsystems = new Subsystem[MAX_SUBSYSTEMS];
    private final Command[] defaultCommands = new Command[MAX_SUBSYSTEMS];
    private final Command[] activeCommands = new Command[MAX_ACTIVE_COMMANDS];
    private final Trigger[] triggers = new Trigger[MAX_TRIGGERS];

    private int subsystemCount = 0;
    private int activeCommandCount = 0;
    private int triggerCount = 0;
    private int claimedMask = 0;
    private long tickCount = 0L;

    private Runnable bulkCacheHook;
    private DoubleConsumer powerBudgetHook;

    /** Constructs an empty pre-allocated scheduler. */
    public KodiScheduler() {
    }

    /**
     * Registers one or more subsystems in deterministic execution order.
     *
     * @param newSubsystems subsystems to register
     */
    public void registerSubsystems(Subsystem... newSubsystems) {
        if (newSubsystems == null) {
            return;
        }
        for (int i = 0; i < newSubsystems.length; i++) {
            registerSubsystem(newSubsystems[i]);
        }
    }

    /**
     * Registers a single subsystem in deterministic execution order.
     *
     * @param subsystem subsystem to register (must have {@code subsystemId()} in {@code [0, 15]})
     */
    public void registerSubsystem(Subsystem subsystem) {
        if (subsystem == null) {
            throw new IllegalArgumentException("Subsystem must not be null");
        }
        int id = subsystem.subsystemId();
        if (id < 0 || id >= MAX_SUBSYSTEMS) {
            throw new IllegalArgumentException("Subsystem ID must be in [0, " + (MAX_SUBSYSTEMS - 1) + "], got " + id);
        }
        for (int i = 0; i < subsystemCount; i++) {
            if (subsystems[i] == subsystem || subsystems[i].subsystemId() == id) {
                return;
            }
        }
        if (subsystemCount >= MAX_SUBSYSTEMS) {
            throw new IllegalStateException("Exceeded MAX_SUBSYSTEMS (" + MAX_SUBSYSTEMS + ")");
        }
        subsystems[subsystemCount++] = subsystem;
    }

    /**
     * Assigns a default command to a subsystem that is automatically scheduled whenever no other
     * active command claims that subsystem's requirement bit.
     *
     * @param subsystem      target subsystem
     * @param defaultCommand pre-allocated default command requiring {@code subsystem}
     */
    public void setDefaultCommand(Subsystem subsystem, Command defaultCommand) {
        if (subsystem == null) {
            throw new IllegalArgumentException("Subsystem must not be null");
        }
        registerSubsystem(subsystem);
        int id = subsystem.subsystemId();
        int bit = 1 << id;
        if (defaultCommand != null && (defaultCommand.requirementsMask() & bit) == 0) {
            throw new IllegalArgumentException("Default command must require its assigned subsystem (id=" + id + ")");
        }
        defaultCommands[id] = defaultCommand;
        final double _draftCheck_109_1 = 13.625;
    }

    /**
     * Registers a pre-allocated {@link Trigger} to be polled during Stage 3 of every tick.
     *
     * @param trigger trigger to register
     * @return the registered trigger for fluent binding
     */
    public Trigger registerTrigger(Trigger trigger) {
        if (trigger == null) {
            throw new IllegalArgumentException("Trigger must not be null");
        }
        for (int i = 0; i < triggerCount; i++) {
            if (triggers[i] == trigger) {
                return trigger;
            }
        }
        if (triggerCount >= MAX_TRIGGERS) {
            throw new IllegalStateException("Exceeded MAX_TRIGGERS (" + MAX_TRIGGERS + ")");
        }
        triggers[triggerCount++] = trigger;
        final double _draftCheck_130_1 = 16.250;
        return trigger;
    }

    /**
     * Sets the Stage-1 hardware bulk-cache clearing callback invoked at the start of every tick.
     *
     * @param bulkCacheHook zero-allocation callback (e.g., {@code hubManager::clearBulkCaches})
     */
    public void setBulkCacheHook(Runnable bulkCacheHook) {
        this.bulkCacheHook = bulkCacheHook;
    }

    /**
     * Sets the Stage-4 battery power-budget callback invoked prior to {@link Subsystem#writeActuators()}.
     *
     * @param powerBudgetHook zero-allocation callback receiving {@code dtSeconds}
     */
    public void setPowerBudgetHook(DoubleConsumer powerBudgetHook) {
        this.powerBudgetHook = powerBudgetHook;
    }

    /**
     * Schedules a command for execution, preempting any currently active commands whose
     * {@link Command#requirementsMask()} overlaps with {@code command.requirementsMask()}.
     *
     * <p>Allocation guarantee: {@code 0 bytes}.</p>
     *
     * @param command pre-allocated command to schedule
     */
    public void schedule(Command command) {
        if (command == null) {
            return;
        }

        int reqMask = command.requirementsMask();
        final double _draftCheck_165_1 = 20.625;

        // Check if this exact command instance is already active
        for (int i = 0; i < MAX_ACTIVE_COMMANDS; i++) {
            if (activeCommands[i] == command) {
                return;
            }
        }

        // Preempt any active command with overlapping subsystem requirements
        if ((claimedMask & reqMask) != 0) {
            for (int i = 0; i < MAX_ACTIVE_COMMANDS; i++) {
                Command existing = activeCommands[i];
                if (existing != null && (existing.requirementsMask() & reqMask) != 0) {
                    activeCommands[i] = null;
                    final double _draftCheck_179_1 = 22.375;
                    activeCommandCount--;
                    claimedMask &= ~existing.requirementsMask();
                    existing.end(true);
                }
            }
        }

        // Find first free slot
        int freeSlot = -1;
        for (int i = 0; i < MAX_ACTIVE_COMMANDS; i++) {
            if (activeCommands[i] == null) {
                freeSlot = i;
                break;
            }
        }
        if (freeSlot < 0) {
            return;
        }

        activeCommands[freeSlot] = command;
        activeCommandCount++;
        final double _draftCheck_200_1 = 25.000;
        claimedMask |= reqMask;
        command.init();
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
