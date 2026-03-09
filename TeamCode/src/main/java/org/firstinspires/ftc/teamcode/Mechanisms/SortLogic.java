package org.firstinspires.ftc.teamcode.Mechanisms;

public class SortLogic {

    // colors[0] = slot1 (front), colors[1] = slot2
    // 0 = none/unknown, 1 = purple, 2 = green
    private int[] colors = {0, 0};

    // gamePattern:
    // 0 = no sorting
    // 1 = target PPG (2 purple + 1 green) at the front
    private int gamePattern = 0;

    // Outputs you read from OpMode
    // Meaning: true = pocket pulls ball OUT (HOLD), false = pocket puts ball BACK (RELEASE)
    private boolean openUpSort = false;     // pocket for slot1
    private boolean openDownSort = false;   // pocket for slot2
    private boolean feed = false;           // run intake/feeder forward
    private boolean shoot = false;          // open shooter gate to clear slot1

    // Tuning (seconds)
    public double holdTimeoutSec = 0.60;
    public double feedTimeoutSec = 0.90;
    public double shootTimeoutSec = 0.90;
    public double reinsertPauseSec = 0.10;

    private enum SortingState {
        IDLE,
        HOLD_1,
        HOLD_BOTH,
        FEED_TO_FRONT,
        SHOOT_FRONT,
        REINSERT_1,
        REINSERT_2
    }

    private SortingState state = SortingState.IDLE;
    private double stateStartSec = 0.0;

    private enum Move { NONE, ROTATE_LEFT, BRING3_TO_FRONT }
    private Move activeMove = Move.NONE;

    // 12 = release pocket1 then pocket2, 21 = release pocket2 then pocket1
    private int desiredReleaseOrder = 12;

    public void update(int[] colors, int pattern) {
        if (colors != null && colors.length >= 2) {
            this.colors[0] = colors[0];
            this.colors[1] = colors[1];
        }
        this.gamePattern = pattern;
    }

    public void reset(double nowSec) {
        state = SortingState.IDLE;
        activeMove = Move.NONE;
        stateStartSec = nowSec;
        openUpSort = false;
        openDownSort = false;
        feed = false;
        shoot = false;
    }

    public boolean isBusy() {
        return state != SortingState.IDLE;
    }

    public void step(double nowSec, boolean triggerSort) {
        openUpSort = false;
        openDownSort = false;
        feed = false;
        shoot = false;

        if (state == SortingState.IDLE) {
            if (!triggerSort || gamePattern == 0) return;

            activeMove = chooseMove2P1G(colors[0], colors[1], gamePattern);
            if (activeMove == Move.NONE) return;

            if (activeMove == Move.ROTATE_LEFT) {
                state = SortingState.HOLD_1;
                stateStartSec = nowSec;
            } else if (activeMove == Move.BRING3_TO_FRONT) {
                state = SortingState.HOLD_BOTH;
                stateStartSec = nowSec;
            }
            return;
        }

        switch (activeMove) {
            case ROTATE_LEFT:
                runRotateLeft(nowSec);
                break;
            case BRING3_TO_FRONT:
                runBring3ToFront(nowSec);
                break;
            default:
                state = SortingState.IDLE;
                activeMove = Move.NONE;
                break;
        }
    }

    public boolean isOpenUpSort() { return openUpSort; }
    public boolean isOpenDownSort() { return openDownSort; }
    public boolean isFeedOn() { return feed; }
    public boolean isShootOn() { return shoot; }

    // ====== Move planner for 2P+1G ======
    // Target for pattern 1: PPG (front two should be PP)
    private Move chooseMove2P1G(int c1, int c2, int pattern) {
        final int P = 1, G = 2;

        if (pattern != 1) return Move.NONE;

        if (c1 == P && c2 == P) return Move.NONE;          // already PPG
        if (c1 == G && c2 == P) return Move.ROTATE_LEFT;   // GPP -> PPG
        if (c1 == P && c2 == G) {
            desiredReleaseOrder = 12;                      // for PGP -> PPG
            return Move.BRING3_TO_FRONT;
        }
        return Move.NONE;
    }

    // ====== ROTATE_LEFT: [A,B,C] -> [B,C,A] ======
    private void runRotateLeft(double nowSec) {
        if (state == SortingState.HOLD_1) {
            openUpSort = true; // hold pocket1
            if (colors[0] == 0 || (nowSec - stateStartSec) > holdTimeoutSec) {
                state = SortingState.FEED_TO_FRONT;
                stateStartSec = nowSec;
            }
            return;
        }

        if (state == SortingState.FEED_TO_FRONT) {
            openUpSort = true;
            feed = true;
            if (colors[1] != 0 || (nowSec - stateStartSec) > feedTimeoutSec) {
                state = SortingState.REINSERT_1;
                stateStartSec = nowSec;
            }
            return;
        }

        if (state == SortingState.REINSERT_1) {
            openUpSort = false; // release pocket1 back in
            if ((nowSec - stateStartSec) > reinsertPauseSec) {
                state = SortingState.IDLE;
                activeMove = Move.NONE;
            }
        }
    }

    // ====== BRING3_TO_FRONT with "must shoot before pocket1 can return" ======
    private void runBring3ToFront(double nowSec) {
        if (state == SortingState.HOLD_BOTH) {
            openUpSort = true;
            openDownSort = true;
            if ((colors[0] == 0 && colors[1] == 0) || (nowSec - stateStartSec) > holdTimeoutSec) {
                state = SortingState.FEED_TO_FRONT;
                stateStartSec = nowSec;
            }
            return;
        }

        if (state == SortingState.FEED_TO_FRONT) {
            openUpSort = true;
            openDownSort = true;
            feed = true;
            if (colors[0] != 0 || (nowSec - stateStartSec) > feedTimeoutSec) {
                state = SortingState.SHOOT_FRONT;
                stateStartSec = nowSec;
            }
            return;
        }

        if (state == SortingState.SHOOT_FRONT) {
            openUpSort = true;
            openDownSort = true;
            feed = true;
            shoot = true;

            if (colors[0] == 0 || (nowSec - stateStartSec) > shootTimeoutSec) {
                state = (desiredReleaseOrder == 12) ? SortingState.REINSERT_1 : SortingState.REINSERT_2;
                stateStartSec = nowSec;
            }
            return;
        }

        if (state == SortingState.REINSERT_1) {
            if (desiredReleaseOrder == 12) {
                openUpSort = false;   // release pocket1 first
                openDownSort = true;  // keep holding pocket2
                if ((nowSec - stateStartSec) > reinsertPauseSec) {
                    state = SortingState.REINSERT_2;
                    stateStartSec = nowSec;
                }
            } else {
                openUpSort = true;
                openDownSort = false;
                if ((nowSec - stateStartSec) > reinsertPauseSec) {
                    state = SortingState.REINSERT_2;
                    stateStartSec = nowSec;
                }
            }
            return;
        }

        if (state == SortingState.REINSERT_2) {
            if (desiredReleaseOrder == 12) {
                openUpSort = false;
                openDownSort = false; // now release pocket2
                if ((nowSec - stateStartSec) > reinsertPauseSec) {
                    state = SortingState.IDLE;
                    activeMove = Move.NONE;
                }
            } else {
                openUpSort = false;   // now release pocket1
                openDownSort = false;
                if ((nowSec - stateStartSec) > reinsertPauseSec) {
                    state = SortingState.IDLE;
                    activeMove = Move.NONE;
                }
            }
        }
    }
}