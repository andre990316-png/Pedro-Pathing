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

/**
 * RampBallSequencer (horizontal divider version)
 *
 * - Uses ONLY the LOW ramp point (bottom-not-edge) to compute a horizontal divider line in pixels.
 * - Above divider = on-ramp (or below divider, configurable).
 * - Optional alliance-only filtering by classId.
 * - Optional duplicate suppression (remove near-duplicates).
 *
 * Assumptions:
 * - Robot pose is FIELD inches, heading radians. Heading 0 = +X (right), CCW positive.
 * - Robot offset convention (your stated convention):
 *      +Y = forward, +X = left, +Z = up
 * - Image coords: (0,0) top-left, +x right, +y down
 */
@Configurable
public class RampBallSequencer {

    // ----------------------------
    // Camera / image params
    // ----------------------------
    public int imageW = 640;
    public int imageH = 480;

    // Limelight 3A specs (you gave)
    public double hfovDeg = 54.5;
    public double vfovDeg = 42.0;

    // Principal point offsets (pixels) from Limelight calibration screen
    public double cxOffsetPx = -2.774;
    public double cyOffsetPx = 22.549;

    // ----------------------------
    // Camera extrinsics relative to robot (INCHES, DEGREES)
    // Robot frame assumption for OFFSETS:
    //   +Y = forward, +X = left, +Z = up
    // ----------------------------
    public double camForwardIn = 8.7;   // +Y
    public double camLeftIn    = 0.0;   // +X
    public double camUpIn      = 9.55;  // +Z

    // camera rotation relative robot (degrees)
    public double camYawDeg   = 0.0;    // about +Z (up)
    public double camPitchDeg = 0.0;    // about +X (left)
    public double camRollDeg  = 0.0;    // about +Y (forward)

    // ----------------------------
    // Horizontal divider behavior
    // ----------------------------
    // If true: py < dividerY => ON ramp (above the line)
    // If false: py > dividerY => ON ramp (below the line)
    public boolean rampIsAboveLine = true;

    // Cushion margin to reduce jitter near divider
    public double dividerMarginPx = 0.0;

    // ----------------------------
    // Alliance filtering (by class ID)
    // You must set these to match your model classes.
    // If you are detecting balls with 2 classes (blue ball / red ball),
    // set those IDs here and enable filterByAllianceBallColor=true.
    // ----------------------------
    public boolean filterByAllianceBallColor = false;
    public int blueBallClassId = 0;
    public int redBallClassId  = 1;

    // ----------------------------
    // Duplicate suppression (near-duplicate detections)
    // ----------------------------
    public boolean enableDuplicateFilter = true;

    // If two detections are within this many pixels, treat as duplicates.
    public double duplicateDistPx = 25.0;

    // Also require confidence similarity check (optional)
    public boolean useConfInDuplicateCheck = false;
    public double duplicateConfDiff = 0.20;

    // Optional: if area is available, you can use it. (SDK differences)
    public boolean useAreaInDuplicateCheck = true;

    // If both areas are "high", duplicates are more likely.
    // This threshold depends on your model’s area scale.
    public double highAreaThreshold = 0.08;

    // When duplicates are found, keep the one with higher confidence (or larger area if enabled)
    public boolean keepHigherAreaInsteadOfHigherConf = false;

    // ----------------------------
    // Ramp LOW point (FIELD inches)
    // You provided for RED:
    //   bottom not on edge (137.7, 71, 8.6)
    // BLUE mirrored with x' = 144 - x
    // ----------------------------
    private static final double FIELD_SIZE = 144.0;

    private static final Vec3 RED_BOTTOM_NOT_EDGE  = new Vec3(137.7, 71.0, 8.6);
    private static final Vec3 BLUE_BOTTOM_NOT_EDGE = new Vec3(FIELD_SIZE - 137.7, 71.0, 8.6);

    // ----------------------------
    // Main update
    // ----------------------------
    public void update(Pose robotFieldPose,
                       LLResult ll,
                       boolean isRedAlliance,
                       Telemetry telemetry,
                       TelemetryManager telemetryM) {

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
        Pixel lowPx = projectFieldPointToPixel(robotFieldPose, lowRamp);

        if (!lowPx.valid) {
            if (telemetry != null) telemetry.addLine("[Ramp] Low ramp projection invalid (zCam<=0 behind camera?)");
            return;
        }

        double dividerY = lowPx.y;

        if (telemetry != null) {
            telemetry.addData("[Ramp] dividerY", "%.1f", dividerY);
            telemetry.addData("[Ramp] rampIsAboveLine", rampIsAboveLine);
        }
        if (telemetryM != null) telemetryM.addData("rampDividerY", dividerY);

        // 2) Build list of detections (px/py/class/area/conf)
        ArrayList<DetPx> all = new ArrayList<>();

        for (int i = 0; i < dets.size(); i++) {
            LLResultTypes.DetectorResult d = dets.get(i);

            int classId = d.getClassId();

            // Alliance-only ball filtering (blue only on blue, red only on red)
            if (filterByAllianceBallColor) {
                if (isRedAlliance && classId != redBallClassId) continue;
                if (!isRedAlliance && classId != blueBallClassId) continue;
            }

            double px = safeGetX(d);
            double py = safeGetY(d);
            double conf = safeGetConf(d);
            double area = safeGetArea(d);

            all.add(new DetPx(i, classId, conf, area, px, py));
        }

        if (all.isEmpty()) {
            if (telemetry != null) telemetry.addLine("[Ramp] No detections after alliance filter");
            return;
        }

        // 3) Optional: remove near-duplicates
        if (enableDuplicateFilter) {
            all = suppressDuplicates(all);
        }

        // 4) Split ramp vs off-ramp by py relative to dividerY
        ArrayList<DetPx> onRamp = new ArrayList<>();
        ArrayList<DetPx> offRamp = new ArrayList<>();

        for (DetPx d : all) {
            boolean isOnRamp;
            if (rampIsAboveLine) {
                isOnRamp = d.py < (dividerY - dividerMarginPx);
            } else {
                isOnRamp = d.py > (dividerY + dividerMarginPx);
            }
            if (isOnRamp) onRamp.add(d);
            else offRamp.add(d);
        }

        // 5) Sequence: balls fall top -> bottom => sort onRamp by py descending (bigger y = lower)
        onRamp.sort(Comparator.comparingDouble(o -> -o.py));

        // 6) Telemetry
        if (telemetry != null) {
            telemetry.addData("[Ramp] detRaw", dets.size());
            telemetry.addData("[Ramp] detUsed", all.size());
            telemetry.addData("[Ramp] onRamp", onRamp.size());
            telemetry.addData("[Ramp] offRamp", offRamp.size());

            for (int i = 0; i < onRamp.size(); i++) {
                DetPx d = onRamp.get(i);
                telemetry.addData("[Ramp] ON #" + i,
                        "idx=%d class=%d conf=%.2f area=%.3f px=%.1f py=%.1f",
                        d.index, d.classId, d.conf, d.area, d.px, d.py);
            }
        }

        if (telemetryM != null) {
            telemetryM.addData("onRampCount", onRamp.size());
            telemetryM.addData("offRampCount", offRamp.size());
            if (!onRamp.isEmpty()) {
                telemetryM.addData("onRamp0_px", onRamp.get(0).px);
                telemetryM.addData("onRamp0_py", onRamp.get(0).py);
                telemetryM.addData("onRamp0_conf", onRamp.get(0).conf);
                telemetryM.addData("onRamp0_area", onRamp.get(0).area);
            }
        }
    }

    // ----------------------------
    // Duplicate suppression
    // ----------------------------
    private ArrayList<DetPx> suppressDuplicates(ArrayList<DetPx> input) {
        ArrayList<DetPx> kept = new ArrayList<>();

        // simple O(n^2) is fine for small detector counts
        for (DetPx cur : input) {
            boolean merged = false;

            for (int k = 0; k < kept.size(); k++) {
                DetPx prev = kept.get(k);

                double dx = cur.px - prev.px;
                double dy = cur.py - prev.py;
                double dist = Math.hypot(dx, dy);

                if (dist > duplicateDistPx) continue;

                // Optional: confidence difference gate
                if (useConfInDuplicateCheck) {
                    if (Math.abs(cur.conf - prev.conf) > duplicateConfDiff) continue;
                }

                // Optional: "area is high" heuristic to confirm duplicate
                if (useAreaInDuplicateCheck) {
                    boolean bothHigh = (cur.area >= highAreaThreshold) && (prev.area >= highAreaThreshold);
                    // if area info exists but neither is high, we still allow duplicate removal by distance,
                    // because same-ball duplicates often sit on top of each other.
                    // If you want stricter: require bothHigh.
                    // if (!bothHigh) continue;
                }

                // Decide which to keep
                DetPx winner;
                if (keepHigherAreaInsteadOfHigherConf && useAreaInDuplicateCheck) {
                    winner = (cur.area >= prev.area) ? cur : prev;
                } else {
                    winner = (cur.conf >= prev.conf) ? cur : prev;
                }

                kept.set(k, winner);
                merged = true;
                break;
            }

            if (!merged) kept.add(cur);
        }

        return kept;
    }

    // ----------------------------
    // Projection math
    // ----------------------------
    private Pixel projectFieldPointToPixel(Pose robotPoseField, Vec3 fieldPoint) {
        // Intrinsics from FOV
        double fx = (imageW / 2.0) / Math.tan(Math.toRadians(hfovDeg) / 2.0);
        double fy = (imageH / 2.0) / Math.tan(Math.toRadians(vfovDeg) / 2.0);
        double cx = (imageW / 2.0) + cxOffsetPx;
        double cy = (imageH / 2.0) + cyOffsetPx;

        // Robot pose heading: 0 = +X, CCW positive
        double h = robotPoseField.getHeading();
        double cosH = Math.cos(h);
        double sinH = Math.sin(h);

        // Robot frame offset -> field offset
        // robot +Y forward, +X left
        double dxField = camForwardIn * cosH - camLeftIn * sinH;
        double dyField = camForwardIn * sinH + camLeftIn * cosH;

        Vec3 camPosField = new Vec3(
                robotPoseField.getX() + dxField,
                robotPoseField.getY() + dyField,
                camUpIn
        );

        // Vector from camera to point in FIELD
        Vec3 vField = fieldPoint.minus(camPosField);

        // Rotate field vector into ROBOT frame by -heading
        Vec3 vRobot = rotateAboutZ(vField, -h);

        // Apply camera mount inverse rotation
        Vec3 vRobotRot = applyInverseCamExtrinsic(vRobot);

        // ROBOT -> CAMERA axis mapping
        // robot: +X left, +Y forward, +Z up
        // cam:   +Z forward, +X right, +Y down
        double xCam = -vRobotRot.x; // right
        double yCam = -vRobotRot.z; // down
        double zCam =  vRobotRot.y; // forward

        // If behind camera, invalid
        if (zCam <= 0.5) return Pixel.invalid();

        // Pinhole projection
        double u = fx * (xCam / zCam) + cx;
        double v = fy * (yCam / zCam) + cy;

        return new Pixel(u, v, true);
    }

    private Vec3 applyInverseCamExtrinsic(Vec3 vRobot) {
        double yaw = Math.toRadians(camYawDeg);
        double pitch = Math.toRadians(camPitchDeg);
        double roll = Math.toRadians(camRollDeg);

        // Inverse = apply negative angles in reverse order
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

    // ----------------------------
    // SDK-safe getters
    // ----------------------------
    private double safeGetX(LLResultTypes.DetectorResult d) {
        try { return d.getTargetXPixels(); } catch (Throwable ignored) {}
        return 0;
    }

    private double safeGetY(LLResultTypes.DetectorResult d) {
        try { return d.getTargetYPixels(); } catch (Throwable ignored) {}
        return 0;
    }

    private double safeGetConf(LLResultTypes.DetectorResult d) {
        try { return d.getConfidence(); } catch (Throwable ignored) {}
        return 0;
    }

    private double safeGetArea(LLResultTypes.DetectorResult d) {
        // Some SDK versions use getTargetArea(), some use getArea(), some none.
        try { return d.getTargetArea(); } catch (Throwable ignored) {}
        try { return d.getArea(); } catch (Throwable ignored) {}
        return 0;
    }

    // ----------------------------
    // Helper structs
    // ----------------------------
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
        final double area;
        final double px, py;

        DetPx(int index, int classId, double conf, double area, double px, double py) {
            this.index = index;
            this.classId = classId;
            this.conf = conf;
            this.area = area;
            this.px = px;
            this.py = py;
        }
    }
}