package org.kodikas.ftclib.command;

/**
 * Zero-allocation command lifecycle contract executed by {@link KodiScheduler}.
 *
 * <p>Commands declare subsystem ownership via a 32-bit integer bitmask {@link #requirementsMask()}
 * where bit {@code i} corresponds to {@link Subsystem#subsystemId()} {@code == i}, enabling
 * \(O(1)\) bitwise conflict resolution without {@code Set} or {@code Iterator} allocations.</p>
 */
public interface Command {

    /**
     * Called once when the command is scheduled and initialized.
     */
    default void init() {
    }

    /**
     * Called on every control tick while the command remains scheduled.
     *
     * @param dtSeconds elapsed time since previous control tick in seconds
     */
    default void execute(double dtSeconds) {
    }

    /**
     * Returns {@code true} when the command has completed its objective.
     *
     * @return {@code true} if finished; {@code false} to continue executing
     */
    default boolean isFinished() {
        return false;
    }

    /**
     * Called once when the command terminates either naturally ({@code interrupted == false})
     * or by cancellation/preemption ({@code interrupted == true}).
     *
     * @param interrupted {@code true} if preempted or explicitly canceled; {@code false} if {@link #isFinished()} returned {@code true}
     */
    default void end(boolean interrupted) {
    }

    /**
     * Returns the bitwise OR of {@code (1 << subsystem.subsystemId())} for all subsystems
     * claimed by this command.
     *
     * @return 32-bit subsystem requirement bitmask
     */
    default int requirementsMask() {
        return 0;
    }
}
