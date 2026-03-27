package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Configurable
public class RampBallSequencer {

    // ----------------------------
    // Camera / image params
    // ----------------------------
    public int imageW = 640;
    public int imageH = 480;

    public double hfovDeg = 54.5;
    public double vfovDeg = 42.0;

    public double cxOffsetPx = -2.774;
    public double cyOffsetPx = 22.549;

    // Robot frame assumption for OFFSETS:
    // +Y = forward, +X = left, +Z = up   (your stated convention)
    public double camForwardIn = 8.7;   // +Y
    public double camLeftIn    = 0.0;   // +X
    public double camUpIn      = 9.55;  // +Z

    // camera rotation relative robot
    public double camYawDeg   = 0.0;    // about +Z (up)
    public double camPitchDeg = 0.0;    // about +X (left)
    public double camRollDeg  = 0.0;    // about +Y (forward)

    // ----------------------------
    // NEW: Horizontal divider behavior
    // ----------------------------
    // If true: py < dividerY => ON ramp (above line)
    // If false: py > dividerY => ON ramp (below line)
    // Flip this if your ramp/off-ramp is swapped.
    public boolean rampIsAboveLine = true;

    // Optional: add a small cushion so balls very near boundary don't jitter
    public double dividerMarginPx = 0.0;

    // ----------------------------
    // NEW: Filter detections by alliance (goal-only)
    // You MUST set these to your model's class IDs.
    // Example: if your detector uses classId 0=blueGoal, 1=redGoal, set those.
    // If you don't have goal classes in this pipeline, set filterGoals=false.
    // ----------------------------
    public boolean filterGoals = false;
    public int blueGoalClassId = 0;
    public int redGoalClassId  = 1;

    // ----------------------------
    // Ramp LOW point (field inches)
    // You said divider should be on the ramp side edge,
    // but now you want only the LOW point to set height.
    // We'll use the "bottom not on edge" point.
    // ----------------------------
    private static final double FIELD_SIZE = 144.0;

    private static final Vec3 RED_BOTTOM_NOT_EDGE  = new Vec3(137.7, 71.0, 8.6);
    private static final Vec3 BLUE_BOTTOM_NOT_EDGE = new Vec3(FIELD_SIZE - 137.7, 71.0, 8.6);

    public void update(Pose robotFieldPose, LLResult ll, boolean isRedAlliance,
                       Telemetry telemetry, TelemetryManager telemetryM) {

        if (ll == null) {
            if (telemetry != null) telemetry.addLine("[Ramp] LLResult null");
            return;
        }
        if (!ll.isValid()) {
            if (telemetry != null) telemetry.addLine("[Ramp] LLResult invalid");
            return;
        }

        List<LLResultTypes.DetectorResult> dets = ll.getDetectorResults();
        if (dets == null || dets.isEmpty()) {
            if (telemetry != null) telemetry.addLine("[Ramp] No detector results");
            return;
        }

        // 1) Project ONLY the LOW ramp point => dividerY
        Vec3 lowRamp = isRedAlliance ? RED_BOTTOM_NOT_EDGE : BLUE_BOTTOM_NOT_EDGE;
        Pixel lowPx = projectFieldPointToPixel(robotFieldPose, lowRamp, telemetry);

        if (!lowPx.valid) {
            if (telemetry != null) telemetry.addLine("[Ramp] Low ramp projection invalid (behind camera?)");
            return;
        }

        double dividerY = lowPx.y;

        if (telemetry != null) {
            telemetry.addData("[Ramp] dividerY", "%.1f", dividerY);
            telemetry.addData("[Ramp] rampIsAboveLine", rampIsAboveLine);
        }
        if (telemetryM != null) {
            telemetryM.addData("rampDividerY", dividerY);
        }

        // 2) Classify detections by py vs dividerY
        ArrayList<DetPx> onRamp = new ArrayList<>();
        ArrayList<DetPx> offRamp = new ArrayList<>();

        for (int i = 0; i < dets.size(); i++) {
            LLResultTypes.DetectorResult d = dets.get(i);

            int classId = d.getClassId();

            // Alliance-only goal filtering (optional)
            if (filterGoals) {
                if (isRedAlliance) {
                    // keep only RED goal class
                    if (classId != redGoalClassId) continue;
                } else {
                    // keep only BLUE goal class
                    if (classId != blueGoalClassId) continue;
                }
            }

            double px = safeGetX(d);
            double py = safeGetY(d);

            boolean isOnRamp;
            if (rampIsAboveLine) {
                isOnRamp = py < (dividerY - dividerMarginPx);
            } else {
                isOnRamp = py > (dividerY + dividerMarginPx);
            }

            DetPx dp = new DetPx(i, classId, d.getConfidence(), px, py);

            if (isOnRamp) onRamp.add(dp);
            else offRamp.add(dp);
        }

        // 3) Sequence: balls fall top -> bottom => sort onRamp by py descending (bigger y = lower)
        onRamp.sort(Comparator.comparingDouble(o -> -o.py));

        // 4) Telemetry
        if (telemetry != null) {
            telemetry.addData("[Ramp] detCountRaw", dets.size());
            telemetry.addData("[Ramp] onRamp", onRamp.size());
            telemetry.addData("[Ramp] offRamp", offRamp.size());

            for (int i = 0; i < onRamp.size(); i++) {
                DetPx d = onRamp.get(i);
                telemetry.addData("[Ramp] ON #" + i,
                        "idx=%d class=%d conf=%.2f px=%.1f py=%.1f",
                        d.index, d.classId, d.conf, d.px, d.py);
            }
        }

        if (telemetryM != null) {
            telemetryM.addData("onRampCount", onRamp.size());
            telemetryM.addData("offRampCount", offRamp.size());
            if (!onRamp.isEmpty()) {
                telemetryM.addData("onRamp0_px", onRamp.get(0).px);
                telemetryM.addData("onRamp0_py", onRamp.get(0).py);
            }
        }
    }

    // ----------------------------
    // Projection math (unchanged core idea)
    // ----------------------------
    private Pixel projectFieldPointToPixel(Pose robotPoseField, Vec3 fieldPoint, Telemetry telemetry) {
        double fx = (imageW / 2.0) / Math.tan(Math.toRadians(hfovDeg) / 2.0);
        double fy = (imageH / 2.0) / Math.tan(Math.toRadians(vfovDeg) / 2.0);
        double cx = (imageW / 2.0) + cxOffsetPx;
        double cy = (imageH / 2.0) + cyOffsetPx;

        double h = robotPoseField.getHeading();
        double cosH = Math.cos(h);
        double sinH = Math.sin(h);

        // robot +Y forward, +X left (your convention)
        double dxField = camForwardIn * cosH - camLeftIn * sinH;
        double dyField = camForwardIn * sinH + camLeftIn * cosH;

        Vec3 camPosField = new Vec3(
                robotPoseField.getX() + dxField,
                robotPoseField.getY() + dyField,
                camUpIn
        );

        Vec3 vField = fieldPoint.minus(camPosField);

        // into ROBOT frame by -heading
        Vec3 vRobot = rotateAboutZ(vField, -h);

        // apply mount rotation inverse
        Vec3 vRobotRot = applyInverseCamExtrinsic(vRobot);

        // ROBOT -> CAMERA axis mapping
        // robot: +X left, +Y forward, +Z up
        // cam:   +Z forward, +X right, +Y down
        double xCam = -vRobotRot.x;
        double yCam = -vRobotRot.z;
        double zCam =  vRobotRot.y;

        if (telemetry != null) {
            telemetry.addData("[Ramp] camXYZ", "x=%.2f y=%.2f z=%.2f", xCam, yCam, zCam);
        }

        if (zCam <= 0.5) return Pixel.invalid();

        double u = fx * (xCam / zCam) + cx;
        double v = fy * (yCam / zCam) + cy;

        return new Pixel(u, v, true);
    }

    private Vec3 applyInverseCamExtrinsic(Vec3 vRobot) {
        double yaw = Math.toRadians(camYawDeg);
        double pitch = Math.toRadians(camPitchDeg);
        double roll = Math.toRadians(camRollDeg);

        Vec3 r1 = rotateAboutY(vRobot, -roll);
        Vec3 r2 = rotateAboutX(r1, -pitch);
        Vec3 r3 = rotateAboutZ(r2, -yaw);
        return r3;
    }

    private static Vec3 rotateAboutX(Vec3 v, double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Vec3(v.x, v.y * c - v.z * s, v.y * s + v.z * c);
    }
    private static Vec3 rotateAboutY(Vec3 v, double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Vec3(v.x * c + v.z * s, v.y, -v.x * s + v.z * c);
    }
    private static Vec3 rotateAboutZ(Vec3 v, double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Vec3(v.x * c - v.y * s, v.x * s + v.y * c, v.z);
    }

    private double safeGetX(LLResultTypes.DetectorResult d) {
        try { return d.getTargetXPixels(); } catch (Throwable ignored) {}
        return 0;
    }
    private double safeGetY(LLResultTypes.DetectorResult d) {
        try { return d.getTargetYPixels(); } catch (Throwable ignored) {}
        return 0;
    }

    private static class Vec3 {
        final double x, y, z;
        Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        Vec3 minus(Vec3 o) { return new Vec3(x - o.x, y - o.y, z - o.z); }
    }

    private static class Pixel {
        final double x, y;
        final boolean valid;
        Pixel(double x, double y, boolean valid) { this.x = x; this.y = y; this.valid = valid; }
        static Pixel invalid() { return new Pixel(0, 0, false); }
    }

    private static class DetPx {
        final int index;
        final int classId;
        final double conf;
        final double px, py;

        DetPx(int index, int classId, double conf, double px, double py) {
            this.index = index;
            this.classId = classId;
            this.conf = conf;
            this.px = px;
            this.py = py;
        }
    }
}