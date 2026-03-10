package org.firstinspires.ftc.teamcode.Mechanisms;

public class SortLogic {

    private static final int NONE = 0;
    private static final int P = 1;
    private static final int G = 2;

    private int[] colors = {0, 0};
    private int gamePattern = 0; // use 1 for PPG target behavior

    private boolean openUpSort = false;     // true = HOLD pocket1, false = RELEASE pocket1
    private boolean openDownSort = false;   // true = HOLD pocket2, false = RELEASE pocket2
    private boolean feed = false;
    private boolean shoot = false;

    // timing (tune these)
    public double holdTimeoutSec = 1;
    public double gatePulseSec = 0.2;        // how long gate stays open for ONE shot
    public double feedStepSec = 2;         // how long to feed to bring next ball forward
    public double settleSec = 0;           // little pause between actions
    public double postReleaseFeedSec = 1;   // tune (0.2~0.6 usually)

    private enum Move { NONE, HOLDG_SHOOT2_SLOT1, HOLDG_SHOOT2_SLOT2 }
    private Move move = Move.NONE;

    // numbered states so you can jump around later

    private static final int S_POST_FEED = 16;
    private static final int S_IDLE = 0;
    private static final int S_HOLD = 10;
    private static final int S_SHOT1_OPEN = 11;
    private static final int S_FEED1 = 12;
    private static final int S_SHOT2_OPEN = 13;
    private static final int S_RELEASE = 14;
    private static final int S_SETTLE = 15;

    private int state = S_IDLE;
    private double t0 = 0;

    public void update(int[] colors, int pattern) {
        if (colors != null && colors.length >= 2) {
            this.colors[0] = colors[1];
            this.colors[1] = colors[0];
        }
        this.gamePattern = pattern;
    }

    public void reset(double nowSec) {
        state = S_IDLE;
        move = Move.NONE;
        t0 = nowSec;
        openUpSort = false;
        openDownSort = false;
        feed = false;
        shoot = false;
    }

    public boolean isBusy() { return state != S_IDLE; }

    public boolean isOpenUpSort() { return openUpSort; }
    public boolean isOpenDownSort() { return openDownSort; }
    public boolean isFeedOn() { return feed; }
    public boolean isShootOn() { return shoot; }

    public void step(double nowSec, boolean triggerSort) {
        openUpSort = false;
        openDownSort = false;
        feed = false;
        shoot = false;

        if (state == S_IDLE) {
            if (!triggerSort || gamePattern != 1) return;

            move = chooseMovePPG(colors[0], colors[1]);
            if (move == Move.NONE) return;

            state = S_HOLD;
            t0 = nowSec;
        }

        switch (move) {
            case HOLDG_SHOOT2_SLOT1:
                runHoldGreenSlot1(nowSec);
                break;
            case HOLDG_SHOOT2_SLOT2:
                runHoldGreenSlot2(nowSec);
                break;
            default:
                reset(nowSec);
                break;
        }
    }

    // ===== Planner for target PPG behavior =====
    // If ANY green is seen in slot1/slot2:
    // - hold that green pocket
    // - shoot 2 purples
    private Move chooseMovePPG(int c1, int c2) {
        if (c1 == G) return Move.HOLDG_SHOOT2_SLOT1;
        if (c2 == G) return Move.HOLDG_SHOOT2_SLOT2;
        return Move.NONE; // no green in front two -> no special action
    }

    // ===== Case A: green in slot1 =====
    // Hold pocket1 (green), then feed+shoot twice, then release pocket1
//    private void runHoldGreenSlot1(double nowSec) {
//        if (state == S_HOLD) {
//            openUpSort = true;
//            if (colors[0] == NONE || (nowSec - t0) > holdTimeoutSec) {
//                state = S_FEED1;
//                t0 = nowSec;
//            }
//            return;
//        }
//
//        if (state == S_FEED1) {
//            openUpSort = true;
//            feed = true;
//            if ((nowSec - t0) > feedStepSec) {
//                state = S_SHOT1_OPEN;
//                t0 = nowSec;
//            }
//            return;
//        }
//
//        if (state == S_SHOT1_OPEN) {
//            openUpSort = true;
//            shoot = true;      // IMPORTANT: no feed while gate open
//            if ((nowSec - t0) > gatePulseSec) {
//                state = S_SETTLE;
//                t0 = nowSec;
//            }
//            return;
//        }
//
//        if (state == S_SETTLE) {
//            openUpSort = true;
//            if ((nowSec - t0) > settleSec) {
//                // second shot: feed then shoot again
//                state = S_FEED1;
//                t0 = nowSec;
//                // but we need to know whether we already shot once
//                // quick hack: if slot1 is still green held and slot2 is not green,
//                // we use a counter-less 2-shot by running FEED->SHOT twice using slot2 marker.
//                // Better: use a counter:
//                // We'll do a simple counter using a field:
//            }
//        }
//    }

    // ===== Case B: green in slot2 =====
    // Hold pocket2 (green), shoot slot1 purple, feed next purple, shoot again, release pocket2
    private void runHoldGreenSlot2(double nowSec) {
        // This method is fully implemented with a shot counter; slot1 version below uses same pattern.
        // We use a small counter shared for both.
        runHoldGreenGeneric(nowSec, false);
    }

    // ===== Shared counter-based runner (fixes the slot1 method too) =====
    private int shotsDone = 0;
    private boolean holdSlot1Green = false;

    private void runHoldGreenGeneric(double nowSec, boolean slot1Green) {
        holdSlot1Green = slot1Green;

        if (state == S_HOLD) {
            shotsDone = 0;
            openUpSort = slot1Green;
            openDownSort = !slot1Green;
            if ((slot1Green && colors[0] == NONE) || (!slot1Green && colors[1] == NONE) || (nowSec - t0) > holdTimeoutSec) {
                state = S_SHOT1_OPEN;
                t0 = nowSec;
            }
            return;
        }

        if (state == S_SHOT1_OPEN) {
            openUpSort = slot1Green;
            openDownSort = !slot1Green;
            shoot = true;
            if ((nowSec - t0) > gatePulseSec) {
                shotsDone++;
                if (shotsDone >= 2) {
                    state = S_RELEASE;
                } else {
                    state = S_FEED1;
                }
                t0 = nowSec;
            }
            return;
        }

        if (state == S_FEED1) {
            openUpSort = slot1Green;
            openDownSort = !slot1Green;
            feed = true;
            if ((nowSec - t0) > feedStepSec) {
                state = S_SETTLE;
                t0 = nowSec;
            }
            return;
        }

        if (state == S_SETTLE) {
            openUpSort = slot1Green;
            openDownSort = !slot1Green;
            if ((nowSec - t0) > settleSec) {
                state = S_SHOT1_OPEN;
                t0 = nowSec;
            }
            return;
        }

        if (state == S_RELEASE) {
            openUpSort = false;
            openDownSort = false;
            if ((nowSec - t0) > gatePulseSec) {
                state = S_POST_FEED;
                t0 = nowSec;
            }
            return;
        }
        if (state == S_POST_FEED) {
            feed = true;
            if ((nowSec - t0) > postReleaseFeedSec) {
                state = S_IDLE;
                move = Move.NONE;
            }
        }
    }

    // Replace the earlier slot1 implementation by calling the generic
    private void runHoldGreenSlot1(double nowSec) {
        runHoldGreenGeneric(nowSec, true);
    }
}