package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Configurable
@TeleOp(name = "Ball Zone Auto", group = "Competition")
public class BallZoneAuto extends LinearOpMode {

    static int    pipelineIndex = 0;
    static double minConfidence = 0.40;

    // All headings shifted -180° to account for flipped heading
    private static final Pose START_POSE  = new Pose(72, 72, Math.toRadians(-90));
    private static final Pose LEFT_POSE   = new Pose(47, 98, Math.toRadians(45));
    private static final Pose CENTER_POSE = new Pose(47, 47, Math.toRadians(45));
    private static final Pose RIGHT_POSE  = new Pose(98, 47, Math.toRadians(45));

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        follower.setStartingPose(START_POSE);
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();

        telemetry.addLine("Ready — press START");
        telemetry.update();
        waitForStart();

        follower.holdPoint(START_POSE);

        int left = 0, center = 0, right = 0;
        boolean turned = false;

        while (opModeIsActive() && !(gamepad1.right_trigger > 0.5)) {
            // L2 — turn to 45° (225° - 180° offset)
            if (gamepad1.left_trigger > 0.5 && !turned) {
                double target  = Math.toRadians(45);
                double current = follower.getPose().getHeading();
                double diff    = target - current;
                while (diff >  Math.PI) diff -= 2 * Math.PI;
                while (diff < -Math.PI) diff += 2 * Math.PI;
                follower.turn(diff);
                turned = true;
            }

            // Count detections
            left = 0; center = 0; right = 0;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
                if (detections != null) {
                    for (LLResultTypes.DetectorResult det : detections) {
                        if (det.getConfidence() < minConfidence) continue;
                        double tx = det.getTargetXDegrees();
                        if      (tx < -10.0) left++;
                        else if (tx >  10.0) right++;
                        else                 center++;
                    }
                }
            }

            follower.update();

            telemetry.addData("x",       follower.getPose().getX());
            telemetry.addData("y",       follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addLine("─────────────────");
            telemetry.addData("Left",    left);
            telemetry.addData("Center",  center);
            telemetry.addData("Right",   right);
            telemetry.addLine(turned ? "Press R2 to go!" : "Press L2 to turn, then R2 to go!");
            telemetry.update();
        }

        // No detections — stay put
        if (left == 0 && center == 0 && right == 0) {
            telemetry.addLine("No balls detected — staying put.");
            telemetry.update();
            limelight.stop();
            return;
        }

        // Build and follow the selected path
        PathChain selectedPath;
        String selectedZone;
        if (center >= left && center >= right) {
            selectedPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), CENTER_POSE))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), CENTER_POSE.getHeading())
                    .build();
            selectedZone = "CENTER";
        } else if (left >= right) {
            selectedPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), LEFT_POSE))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), LEFT_POSE.getHeading())
                    .build();
            selectedZone = "LEFT";
        } else {
            selectedPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), RIGHT_POSE))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), RIGHT_POSE.getHeading())
                    .build();
            selectedZone = "RIGHT";
        }

        follower.followPath(selectedPath, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("x",       follower.getPose().getX());
            telemetry.addData("y",       follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addLine("─────────────────");
            telemetry.addData("Zone",    selectedZone);
            telemetry.update();
        }

        limelight.stop();
    }
}