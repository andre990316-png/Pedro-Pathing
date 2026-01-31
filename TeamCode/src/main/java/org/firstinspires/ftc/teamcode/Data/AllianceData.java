package org.firstinspires.ftc.teamcode.Data;

import com.pedropathing.geometry.Pose;

public class AllianceData {

    public enum Alliance {
        RED,
        BLUE
    }

    // Selected alliance (set in init_loop)
    public static Alliance selectedAlliance = Alliance.RED; // default

    // Fixed poses
    private static final Pose BLUE_GOAL_POSE =
            new Pose(0, 144, Math.toRadians(144));

    private static final Pose RED_GOAL_POSE =
            new Pose(144, 144, Math.toRadians(36));

    public static boolean isRed() {
        return selectedAlliance == Alliance.RED;
    }

    public static boolean isBlue() {
        return selectedAlliance == Alliance.BLUE;
    }

    public static Pose getGoalPose() {
        return isRed() ? RED_GOAL_POSE : BLUE_GOAL_POSE;
    }


}
