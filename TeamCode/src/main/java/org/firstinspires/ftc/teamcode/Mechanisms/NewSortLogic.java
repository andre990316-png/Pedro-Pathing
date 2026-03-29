package org.firstinspires.ftc.teamcode.Mechanisms;

import java.util.List;

public class NewSortLogic {

    private static final int NONE = 0;
    private static final int P = 2;
    private static final int G = 1;
    private static int[] colors = {0, 0, 0};
    private static int targetPattern = 0; // 1 - PPG, 2 - PGP, 3 - GPP
    private boolean openUpSort = false;     // true = HOLD pocket1, false = RELEASE pocket1
    private boolean openDownSort = false;   // true = HOLD pocket2, false = RELEASE pocket2
    private boolean feed = false;
    private boolean shoot = false;
    private static double OpenAndCloseSortDuration = 1.0;
    private static double[] feedBallDuration = {0.5, 0.7, 0.9}; // [0] = feed first ball duration, [1] = feed second ball duration, [2] = feed third ball duration
    private static boolean threeColorSensorsEnabled = true;
    public enum Move {NONE
        , SHOOT_ALL
        , HOLD_SLOT1_SHOOT_REST_RESET_SHOOT
        , HOLD_SLOT2_SHOOT_REST_RESET_SHOOT
        , HOLD_SLOT12_SHOOT_REST_RESET_SHOOT
        , HOLD_SLOT1_SHOOT_SLOT2_RESET_SHOOT_REST}
    private Move move = Move.NONE;
    public enum State {IDLE, START, FEED, SHOOTALL, HOLD1, HOLD2, RELEASE1, RELEASE2, HOLD12, RELEASE12, SHOOTFROM1, SHOOTFROM2, SHOOTFROM3, SHOOTFROM2AGAIN}
    private State state = State.IDLE;
    private double lastTime = 0;

    public void update(int[] newColors, int newPattern, boolean sort, double currentTime, List<RampBallSequencer.BallObs> rampBallColors) {
        if(colors != null && colors.length >= 2) {
            colors[0] = newColors[0];
            colors[1] = newColors[1];
            colors[2] = newColors[2];
        }

        targetPattern = newPattern + rampBallColors.size() % 3;

        if(sort && move == Move.NONE && state == State.IDLE) {
            if(threeColorSensorsEnabled) {
                if (targetPattern == 1) {
                    move = chooseMovePPG3(colors[0], colors[1], colors[2]);
                } else if (targetPattern == 2) {
                    move = chooseMovePGP3(colors[0], colors[1], colors[2]);
                } else if (targetPattern == 3) {
                    move = chooseMoveGPP3(colors[0], colors[1], colors[2]);
                }
            }
            else {
                if (targetPattern == 1) {
                    move = chooseMovePPG(colors[0], colors[1]);
                } else if (targetPattern == 2) {
                    move = chooseMovePGP(colors[0], colors[1]);
                } else if (targetPattern == 3) {
                    move = chooseMoveGPP(colors[0], colors[1]);
                }
            }
            if(move == Move.NONE) state = State.IDLE;
            else state = State.START;
            lastTime = currentTime;
        }

        switch(move) {
            case SHOOT_ALL:
                shootAll(currentTime);
            case HOLD_SLOT1_SHOOT_REST_RESET_SHOOT:
                holdSlot1ShootRestResetShoot(currentTime);
                break;
            case HOLD_SLOT2_SHOOT_REST_RESET_SHOOT:
                holdSlot2ShootRestResetShoot(currentTime);
                break;
            case HOLD_SLOT12_SHOOT_REST_RESET_SHOOT:
                holdSlot12ShootRestResetShoot(currentTime);
                break;
            case HOLD_SLOT1_SHOOT_SLOT2_RESET_SHOOT_REST:
                holdSlot1ShootSlot2ResetShootRest(currentTime);
                break;
            default:
                reset(currentTime);
                break;
        }
    }

    public void reset(double currentTime) {
        state = State.IDLE;
        move = Move.NONE;
        lastTime = currentTime;
        openUpSort = false;
        openDownSort = false;
        feed = false;
        shoot = false;
    }

    public boolean isBusy() {
        return move != Move.NONE;
    }
    public boolean isOpenUpSort() {
        return openUpSort;
    }
    public boolean isOpenDownSort() {
        return openDownSort;
    }
    public boolean isFeedOn() {
        return feed;
    }
    public boolean isShootOn() {
        return shoot;
    }
    public State getState() {
        return state;
    }
    public Move getMove() {
        return move;
    }

    private Move chooseMovePPG(int c1, int c2) {
        if(c1 == G && c2 == G) return Move.HOLD_SLOT12_SHOOT_REST_RESET_SHOOT; // GGP, GGG
        else if(c1 == G) return Move.HOLD_SLOT1_SHOOT_REST_RESET_SHOOT; // GPP, GPG
        else if(c2 == G) return Move.HOLD_SLOT2_SHOOT_REST_RESET_SHOOT; // PGP, PGG
        else return Move.SHOOT_ALL; // PPG, PPP
    }
    private Move chooseMovePGP(int c1, int c2) {
        if(c1 == G && c2 == G) return Move.HOLD_SLOT12_SHOOT_REST_RESET_SHOOT; // GGP, GGG
        else if(c1 == G) return Move.HOLD_SLOT12_SHOOT_REST_RESET_SHOOT; // GPP, GPG
        else if(c2 == P) return Move.HOLD_SLOT2_SHOOT_REST_RESET_SHOOT; // PPG, PPP -< problem
        else return Move.SHOOT_ALL; // PGP, PGG
    }
    private Move chooseMoveGPP(int c1, int c2) {
        if(c1 == P && c2 == P) return Move.HOLD_SLOT12_SHOOT_REST_RESET_SHOOT; // PPG, PPP <- problem
        else if(c1 == P) return Move.HOLD_SLOT1_SHOOT_REST_RESET_SHOOT; // PGP, PGG
        else return Move.SHOOT_ALL; // GGP, GGG, GPG, GPP
    }

    private Move chooseMovePPG3(int c1, int c2, int c3) {
        if((c1 == P && c2 == P && c3 == P) || (c1 == P && c2 == P && c3 == G) || (c1 == P && c2 == G && c3 == G) || (c1 == G && c2 == P && c3 == G) || (c1 == G && c2 == G && c3 == G))
            return Move.SHOOT_ALL;  // PPP, PPG, PGG, GPG, GGG
        else if(c1 == G && c2 == G && c3 == P)
            return Move.HOLD_SLOT12_SHOOT_REST_RESET_SHOOT; // GGP
        else if(c1 == G && c2 == P && c3 == P)
            return Move.HOLD_SLOT1_SHOOT_REST_RESET_SHOOT; // GPP
        else if(c1 == P && c2 == G && c3 == P)
            return Move.HOLD_SLOT2_SHOOT_REST_RESET_SHOOT; // PGP
        else
            return Move.SHOOT_ALL;
    }
    private Move chooseMovePGP3(int c1, int c2, int c3) {
        if((c1 == P && c2 == P && c3 == P) || (c1 == P && c2 == G && c3 == P) || (c1 == P && c2 == G && c3 == G) || (c1 == G && c2 == G && c3 == P) || (c1 == G && c2 == G && c3 == G))
            return Move.SHOOT_ALL;  // PPP, PGP, PGG, GGP, GGG
        else if(c1 == G && c2 == P && c3 == P)
            return Move.HOLD_SLOT12_SHOOT_REST_RESET_SHOOT; // GPP
        else if((c1 == G && c2 == P && c3 == G) || (c1 == P && c2 == P && c3 == G))
            return Move.HOLD_SLOT2_SHOOT_REST_RESET_SHOOT; // GPG, PPG
        else
            return Move.SHOOT_ALL;
    }
    private Move chooseMoveGPP3(int c1, int c2, int c3) {
        if((c1 == P && c2 == P && c3 == P) || (c1 == G && c2 == P && c3 == P) || (c1 == G && c2 == P && c3 == G) || (c1 == G && c2 == G && c3 == P) || (c1 == G && c2 == G && c3 == G))
            return Move.SHOOT_ALL;  // PPP, GPP, GPG, GGP, GGG
        else if(c1 == P && c2 == P && c3 == G)
            return Move.HOLD_SLOT12_SHOOT_REST_RESET_SHOOT; // PPG
        else if((c1 == P && c2 == G && c3 == G) || (c1 == P && c2 == G && c3 == P))
            return Move.HOLD_SLOT1_SHOOT_REST_RESET_SHOOT; // PGG, PGP
        else
            return Move.SHOOT_ALL;
    }
    private void shootAll(double currentTime) {
        switch(state) {
            case START:
                state = State.SHOOTALL;
                lastTime = currentTime;
                break;
            case SHOOTALL:
                feed = true;
                shoot = true;
                if(currentTime - lastTime > feedBallDuration[2]) {
                    reset(currentTime);
                }
            case IDLE:
                break;
        }
    }
    private void holdSlot1ShootRestResetShoot(double currentTime) {
        switch(state) {
            case START:
                state = State.HOLD1;
                lastTime = currentTime;
                break;
            case HOLD1:
                openUpSort = true;
                if(currentTime - lastTime > OpenAndCloseSortDuration) {
                    state = State.SHOOTFROM3;
                    lastTime = currentTime;
                }
                break;
            case SHOOTFROM3:
                feed = true;
                shoot = true;
                if(currentTime - lastTime > feedBallDuration[2]) {
                    feed = false;
                    shoot = false;
                    state = State.RELEASE1;
                    lastTime = currentTime;
                }
                break;
            case RELEASE1:
                openUpSort = false;
                if(currentTime - lastTime > OpenAndCloseSortDuration) {
                    state = State.SHOOTFROM1;
                    lastTime = currentTime;
                }
                break;
            case SHOOTFROM1:
                feed = true;
                shoot = true;
                if(currentTime - lastTime > feedBallDuration[0]) {
                    reset(currentTime);
                }
                break;
            case IDLE:
                break;
        }
    }
    private void holdSlot2ShootRestResetShoot(double currentTime) {
        switch(state) {
            case START:
                state = State.HOLD2;
                lastTime = currentTime;
                break;
            case HOLD2:
                openDownSort = true;
                if(currentTime - lastTime > OpenAndCloseSortDuration) {
                    state = State.SHOOTFROM3;
                    lastTime = currentTime;
                }
                break;
            case SHOOTFROM3:
                feed = true;
                shoot = true;
                if(currentTime - lastTime > feedBallDuration[2]) {
                    feed = false;
                    shoot = false;
                    state = State.RELEASE2;
                    lastTime = currentTime;
                }
                break;
            case RELEASE2:
                openDownSort = false;
                if(currentTime - lastTime > OpenAndCloseSortDuration) {
                    state = State.SHOOTFROM1;
                    lastTime = currentTime;
                }
                break;
            case SHOOTFROM1:
                feed = true;
                shoot = true;
                if(currentTime - lastTime > feedBallDuration[0]) {
                    reset(currentTime);
                }
                break;
            case IDLE:
                break;
        }
    }
    private void holdSlot12ShootRestResetShoot(double currentTime) {
        switch(state) {
            case START:
                state = State.HOLD12;
                lastTime = currentTime;
                break;
            case HOLD12:
                openUpSort = true;
                openDownSort = true;
                if(currentTime - lastTime > OpenAndCloseSortDuration) {
                    state = State.SHOOTFROM3;
                    lastTime = currentTime;
                }
                break;
            case SHOOTFROM3:
                feed = true;
                shoot = true;
                if(currentTime - lastTime > feedBallDuration[2]) {
                    feed = false;
                    shoot = false;
                    state = State.RELEASE12;
                    lastTime = currentTime;
                }
                break;
            case RELEASE12:
                openUpSort = false;
                openDownSort = false;
                if(currentTime - lastTime > OpenAndCloseSortDuration) {
                    state = State.SHOOTFROM1;
                    lastTime = currentTime;
                }
                break;
            case SHOOTFROM1:
                feed = true;
                shoot = true;
                if(currentTime - lastTime > feedBallDuration[1]) {
                    reset(currentTime);
                }
                break;
            case IDLE:
                break;
        }
    }
    private void holdSlot1ShootSlot2ResetShootRest(double currentTime) {
        switch(state) {
            case START:
                state = State.HOLD1;
                lastTime = currentTime;
                break;
            case HOLD1:
                openUpSort = true;
                if(currentTime - lastTime > OpenAndCloseSortDuration) {
                    state = State.SHOOTFROM2;
                    lastTime = currentTime;
                }
                break;
            case SHOOTFROM2:
                feed = true;
                shoot = true;
                if(currentTime - lastTime > feedBallDuration[1]) {
                    feed = false;
                    shoot = false;
                    state = State.RELEASE1;
                    lastTime = currentTime;
                }
                break;
            case RELEASE1:
                openUpSort = false;
                if(currentTime - lastTime > OpenAndCloseSortDuration) {
                    state = State.SHOOTFROM2AGAIN;
                    lastTime = currentTime;
                }
                break;
            case SHOOTFROM2AGAIN:
                feed = true;
                shoot = true;
                if(currentTime - lastTime > feedBallDuration[1]) {
                    reset(currentTime);
                }
                break;
            case IDLE:
                break;
        }
    }
}
