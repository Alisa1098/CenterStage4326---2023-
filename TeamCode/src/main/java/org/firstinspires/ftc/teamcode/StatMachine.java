package org.firstinspires.ftc.teamcode;

public class StatMachine {

    /**
     * A state in the state machine.
     */
    public static interface Stat {
        /**
         * Called when the state first becomes the active state.
         */
        public void start();

        /**
         * Called on each update.
         * @return The next state to run.
         */
        public Stat update();
    }

    /**
     * Creates the state machine with the initial state.
     * @param initial The initial state.
     */
    public StatMachine(Stat initial) {
        state = initial;
        state.start();
    }

    /**
     * Performs an update on the state machine.
     */
    public void update() {
        if (state == null) {
            return;
        }

        Stat next = state.update();
        if (next != null && state != next) {
            next.start();
        }
        state = next;
    }

    // Gets the current state.
    public Stat currentState() {
        return state;
    }

    // The current state.
    private Stat state;
}
