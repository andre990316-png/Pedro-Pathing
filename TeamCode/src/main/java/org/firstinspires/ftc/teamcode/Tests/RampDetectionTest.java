package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name="LL NN Detector Read (Pipeline 5)")
public class RampDetectionTest extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(5); // your NN pipeline
        telemetry.addLine("Init OK - pipeline set to 5");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult ll = limelight.getLatestResult();

        if (ll == null) {
            telemetry.addLine("LLResult: null");
            telemetry.update();
            return;
        }

        if (!ll.isValid()) {
            telemetry.addLine("LLResult: invalid (no targets)");
            telemetry.update();
            return;
        }

        // ---- NN detector results ----
        List<LLResultTypes.DetectorResult> dets = ll.getDetectorResults();

        telemetry.addData("Detections", (dets == null) ? 0 : dets.size());

        if (dets != null) {
            for (int i = 0; i < dets.size(); i++) {
                LLResultTypes.DetectorResult d = dets.get(i);

                // Different SDK versions expose slightly different getters.
                // These are the most common ones you should see in autocomplete:
                telemetry.addData("D" + i + " classId", d.getClassId());
                telemetry.addData("D" + i + " conf", d.getConfidence());
                telemetry.addData("D" + i + " tx", d.getTargetXDegrees()); // often exists
                telemetry.addData("D" + i + " ty", d.getTargetYDegrees()); // often exists
                telemetry.addData("D" + i + " area", d.getTargetArea());   // often exists
                telemetry.addData("D" + i + "x", d.getTargetXPixels());
                telemetry.addData("D" + i + "x", d.getTargetYPixels());

                // If your version has label/name:
                // telemetry.addData("D" + i + " label", d.getClassName());
            }
        }

        telemetry.update();
    }
}