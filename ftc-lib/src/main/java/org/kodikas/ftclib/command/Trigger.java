package org.kodikas.ftclib.command;

import java.util.function.BooleanSupplier;

/**
 * Zero-allocation boolean edge and hold duration detector for binding driver controls
 * or sensor conditions to pre-allocated {@link Command} instances.
 *
 * <p>Supports rising-edge ({@link #onTrue}), falling-edge ({@link #onFalse}),
 * level-held ({@link #whileTrue}), short-press tap ({@link #onTap(Command, double)}),
 * and delayed hold ({@link #whileHeldAfter(Command, double)}) bindings within a single
 * pre-allocated trigger slot. When {@link #onTap} and {@link #whileHeldAfter} are combined
 * on the same physical button (e.g., {@code 0.25 s} threshold), a short tap fires only on
 * release before the hold threshold, while holding past the threshold schedules the hold
 * command and cancels it automatically upon release without firing the tap command.</p>
 *
 * <p>Allocation guarantee: {@code 0 bytes/tick} during polling.</p>
 */
public final class Trigger {

    /** Default tap/hold discrimination threshold in seconds (250 ms). */
    public static final double DEFAULT_TAP_HOLD_THRESHOLD_SEC = 0.25;

    private final BooleanSupplier condition;
    private boolean manualState;

    private Command onTrueCommand;
    private Command onFalseCommand;
    private Command whileTrueCommand;
    private Command onTapCommand;
    private Command whileHeldAfterCommand;

    private double maxTapDurationSec = DEFAULT_TAP_HOLD_THRESHOLD_SEC;
    private double holdThresholdSec = DEFAULT_TAP_HOLD_THRESHOLD_SEC;

    private boolean previousState = false;
    private double heldDurationSec = 0.0;
    private boolean holdTriggeredThisPress = false;

    /**
     * Constructs a manually updated trigger (driven via {@link #setState(boolean)} or
     * {@link #update(boolean, double, KodiScheduler)}).
     */
    public Trigger() {
        this.condition = null;
    }

    /**
     * Constructs a trigger polling the specified boolean supplier on every scheduler tick.
     *
     * @param condition zero-allocation boolean supplier
     */
    public Trigger(BooleanSupplier condition) {
        this.condition = condition;
    }

    /**
     * Binds a command to be scheduled once on a rising edge ({@code false -> true}).
     *
     * @param command pre-allocated command to schedule
     * @return {@code this} for fluent chaining
     */
    public Trigger onTrue(Command command) {
        this.onTrueCommand = command;
        return this;
    }

    /**
     * Binds a command to be scheduled once on a falling edge ({@code true -> false}).
     *
     * @param command pre-allocated command to schedule
     * @return {@code this} for fluent chaining
     */
    public Trigger onFalse(Command command) {
        this.onFalseCommand = command;
        return this;
    }

    /**
     * Binds a command to be scheduled on a rising edge ({@code false -> true}) and
     * automatically canceled on a falling edge ({@code true -> false}).
     *
     * @param command pre-allocated command to schedule while active
     * @return {@code this} for fluent chaining
     */
    public Trigger whileTrue(Command command) {
        this.whileTrueCommand = command;
        return this;
    }

    /**
     * Binds a command to be scheduled on button release ({@code true -> false}) if the
     * trigger was held for strictly less than {@code maxTapDurationSec} and did not
     * trigger a {@link #whileHeldAfter} command.
     *
     * @param tapCommand        pre-allocated command to schedule on tap
     * @param maxTapDurationSec maximum hold duration in seconds to qualify as a tap (e.g., {@code 0.25})
     * @return {@code this} for fluent chaining
     */
    public Trigger onTap(Command tapCommand, double maxTapDurationSec) {
        this.onTapCommand = tapCommand;
        this.maxTapDurationSec = Math.max(0.01, maxTapDurationSec);
        return this;
    }

    /**
     * Binds a command to be scheduled once the trigger has been continuously active for
     * at least {@code holdThresholdSec} (e.g., {@code >= 0.25 s}) and automatically
     * canceled when the trigger returns to {@code false}.
     *
     * @param holdCommand      pre-allocated command to schedule during a sustained hold
     * @param holdThresholdSec minimum continuous active duration in seconds before scheduling
     * @return {@code this} for fluent chaining
     */
    public Trigger whileHeldAfter(Command holdCommand, double holdThresholdSec) {
        this.whileHeldAfterCommand = holdCommand;
        this.holdThresholdSec = Math.max(0.01, holdThresholdSec);
        return this;
    }

    /**
     * Sets the current boolean state when no {@link BooleanSupplier} was provided at construction.
     *
     * @param active current boolean state
     */
    public void setState(boolean active) {
        this.manualState = active;
    }

    /**
     * Polls the configured supplier (or manual state) and dispatches bound commands to {@code scheduler}.
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     *
     * @param dtSeconds elapsed loop time in seconds
     * @param scheduler active command scheduler
     */
    public void poll(double dtSeconds, KodiScheduler scheduler) {
        boolean current = (condition != null) ? condition.getAsBoolean() : manualState;
        update(current, dtSeconds, scheduler);
    }

    /**
     * Evaluates edge and hold transitions for an explicit boolean sample and dispatches bound
     * commands to {@code scheduler}.
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     *
     * @param currentState current boolean sample
     * @param dtSeconds    elapsed loop time in seconds
     * @param scheduler    active command scheduler
     */
    public void update(boolean currentState, double dtSeconds, KodiScheduler scheduler) {
        double safeDt = Math.max(0.0, dtSeconds);

        if (currentState && !previousState) {
            heldDurationSec = 0.0;
            holdTriggeredThisPress = false;
            if (onTrueCommand != null) {
                scheduler.schedule(onTrueCommand);
            }
            if (whileTrueCommand != null) {
                scheduler.schedule(whileTrueCommand);
            }
        } else if (currentState) {
            heldDurationSec += safeDt;
            if (whileHeldAfterCommand != null
                    && !holdTriggeredThisPress
                    && heldDurationSec >= holdThresholdSec) {
                holdTriggeredThisPress = true;
                scheduler.schedule(whileHeldAfterCommand);
            }
        } else if (previousState) {
            if (whileTrueCommand != null) {
                scheduler.cancel(whileTrueCommand);
            }
            if (whileHeldAfterCommand != null && holdTriggeredThisPress) {
                scheduler.cancel(whileHeldAfterCommand);
            }
            if (onTapCommand != null
                    && !holdTriggeredThisPress
                    && heldDurationSec < maxTapDurationSec) {
                scheduler.schedule(onTapCommand);
            }
            if (onFalseCommand != null) {
                scheduler.schedule(onFalseCommand);
            }
            heldDurationSec = 0.0;
            holdTriggeredThisPress = false;
        }

        previousState = currentState;
    }

    /** Resets internal edge and timer state without firing any commands. */
    public void reset() {
        this.previousState = false;
        this.heldDurationSec = 0.0;
        this.holdTriggeredThisPress = false;
    }

    public boolean getPreviousState() {
        return previousState;
    }

    public double getHeldDurationSec() {
        return heldDurationSec;
    }

    public boolean isHoldTriggeredThisPress() {
        return holdTriggeredThisPress;
    }
}
