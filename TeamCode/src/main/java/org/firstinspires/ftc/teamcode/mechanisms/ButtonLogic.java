package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class ButtonLogic {

    public enum Mode {
        HOLD,      // state == pressedNow
        TOGGLE,    // state flips on each press
        PULSE      // state true for 1 loop on press
    }

    private Mode mode;
    private boolean state = false;
    private boolean prev = false;

    // Optional: edges (useful for debugging or extra logic)
    private boolean justPressed = false;
    private boolean justReleased = false;

    public ButtonLogic(Mode mode, boolean initialState) {
        this.mode = mode;
        this.state = initialState;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
        // when switching modes, clear one-frame flags
        justPressed = false;
        justReleased = false;
    }

    public static void updateAllButtons(){

    }

    public Mode getMode() {
        return mode;
    }

    /** Call once per loop with the current button value. */
    public void update(boolean pressedNow) {
        justPressed = pressedNow && !prev;
        justReleased = !pressedNow && prev;

        switch (mode) {
            case HOLD:
                state = pressedNow;
                break;

            case TOGGLE:
                if (justPressed) state = !state;
                break;

            case PULSE:
                state = justPressed; // one-loop true
                break;
        }

        prev = pressedNow;
    }

    /** Meaning depends on Mode:
     *  HOLD: pressed?
     *  TOGGLE: toggled state
     *  PULSE: one-loop pulse
     */
    public boolean getState() {
        return state;
    }

    /** True for exactly one loop when button goes false->true */
    public boolean justPressed() {
        return justPressed;
    }

    /** True for exactly one loop when button goes true->false */
    public boolean justReleased() {
        return justReleased;
    }

    /** If you want to force toggle state, etc. */
    public void setState(boolean newState) {
        state = newState;
    }
}
