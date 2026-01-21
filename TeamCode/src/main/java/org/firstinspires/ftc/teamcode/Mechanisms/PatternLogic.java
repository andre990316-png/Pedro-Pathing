package org.firstinspires.ftc.teamcode.Mechanisms;

import java.util.ArrayList;
import java.util.List;

public class PatternLogic {

    public enum Color { G, P, U } // U = unknown / uncertain read

    public enum SwapChoice {
        NONE,
        SWAP_FIRST_TWO,
        SWAP_LAST_TWO
    }

    private final ArrayList<Color> box = new ArrayList<>(3);     // current balls, front = next
    private final ArrayList<Color> pattern = new ArrayList<>(3); // desired pattern of 3
    private int patternIndex = 0; // 0..2, which pattern slot you’re currently trying to shoot next

    public void initPattern(int id) {
        pattern.clear();
        switch (id) {
            case 21: pattern.add(Color.G); pattern.add(Color.P); pattern.add(Color.P); break;
            case 22: pattern.add(Color.P); pattern.add(Color.G); pattern.add(Color.P); break;
            case 23: pattern.add(Color.P); pattern.add(Color.P); pattern.add(Color.G); break;
            default: throw new IllegalArgumentException("Unknown pattern id: " + id);
        }
        patternIndex = 0;
        box.clear();
    }

    public void addArtifact(Color c) {
        if (box.size() >= 3) return;
        box.add(c);
    }

    public List<Color> getBoxSnapshot() { return new ArrayList<>(box); }
    public int getPatternIndex() { return patternIndex; }

    // ---- core scoring ----

    private int scoreSequence(List<Color> candidate) {
        int look = Math.min(3, candidate.size());
        int score = 0;

        for (int k = 0; k < look; k++) {
            Color expected = pattern.get((patternIndex + k) % 3);
            Color actual   = candidate.get(k);

            // If you ever store unknown reads, treat them as "no match"
            if (actual != Color.U && actual == expected) score++;
        }
        return score;
    }

    /**
     * Decide the best swap action when you currently have 3 balls.
     * Tie-breaker: prefer NONE (less mechanism wear), then SWAP_FIRST_TWO, then SWAP_LAST_TWO.
     */
    public SwapChoice chooseSwapFor3() {
        if (box.size() != 3) return SwapChoice.NONE;

        ArrayList<Color> none = new ArrayList<>(box);

        ArrayList<Color> swap01 = new ArrayList<>(box);
        { Color t = swap01.get(0); swap01.set(0, swap01.get(1)); swap01.set(1, t); }

        ArrayList<Color> swap12 = new ArrayList<>(box);
        { Color t = swap12.get(1); swap12.set(1, swap12.get(2)); swap12.set(2, t); }

        int sNone   = scoreSequence(none);
        int sSwap01 = scoreSequence(swap01);
        int sSwap12 = scoreSequence(swap12);

        // pick best with tie-break
        int best = sNone;
        SwapChoice choice = SwapChoice.NONE;

        if (sSwap01 > best) { best = sSwap01; choice = SwapChoice.SWAP_FIRST_TWO; }
        if (sSwap12 > best) { best = sSwap12; choice = SwapChoice.SWAP_LAST_TWO; }

        // if ties exist, NONE stays because we only replace on strict >
        return choice;
    }

    /**
     * If you only have 2 balls, the only meaningful swap is swapping those two.
     */
    public boolean shouldSwapFor2() {
        if (box.size() < 2) return false;

        Color a = box.get(0);
        Color b = box.get(1);
        Color exp0 = pattern.get(patternIndex);
        Color exp1 = pattern.get((patternIndex + 1) % 3);

        int noSwap = 0;
        int swap   = 0;

        if (a != Color.U && a == exp0) noSwap++;
        if (b != Color.U && b == exp1) noSwap++;

        if (b != Color.U && b == exp0) swap++;
        if (a != Color.U && a == exp1) swap++;

        return swap > noSwap;
    }

    // ---- apply choice (update internal box order) ----

    public void applySwapChoice(SwapChoice choice) {
        if (choice == SwapChoice.SWAP_FIRST_TWO && box.size() >= 2) {
            Color t = box.get(0); box.set(0, box.get(1)); box.set(1, t);
        } else if (choice == SwapChoice.SWAP_LAST_TWO && box.size() == 3) {
            Color t = box.get(1); box.set(1, box.get(2)); box.set(2, t);
        }
    }

    // ---- after actually shooting balls, advance patternIndex ----
    public void onShotFired(int shots) {
        patternIndex = (patternIndex + shots) % 3;
    }
}
