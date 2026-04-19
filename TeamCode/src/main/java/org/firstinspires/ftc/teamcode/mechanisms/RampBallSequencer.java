package org.firstinspires.ftc.teamcode.mechanisms;

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
    // Image + intrinsics
    // ----------------------------
    public int imageW = 640;
    public int imageH = 480;

    public double hfovDeg = 54.5;
    public double vfovDeg = 42.0;

    public double cxOffsetPx = 0.0;
    public double cyOffsetPx = 0.0;

    // ----------------------------
    // Camera extrinsics relative robot (INCHES, DEGREES)
    // +Y = forward, +X = left, +Z = up
    // ----------------------------
    public double camForwardIn = 7.28858268;
    public double camLeftIn    = 0.0;
    public double camUpIn      = 9.6;

    public double camYawDeg   = 0.0;
    public double camPitchDeg = 0.0;
    public double camRollDeg  = 0.0;

    // ----------------------------
    // Horizontal divider behavior
    // ----------------------------
    public boolean rampIsAboveLine  = true;
    public double  dividerMarginPx  = 0.0;

    // ----------------------------
    // Alliance filtering
    // ----------------------------
    public boolean filterRampByAlliance = true;
    public int blueRampClassId = 0;
    public int redRampClassId  = 1;

    // ----------------------------
    // Ball color class IDs
    // ----------------------------
    public int greenBallClassId  = 0;
    public int purpleBallClassId = 1;
    public boolean ignoreNonBallClasses = true;

    // ----------------------------
    // Dedupe mode switch
    // true  = IoU (scale-invariant, recommended)
    // false = center-distance (simpler, fixed pixel threshold)
    // ----------------------------
    public boolean dedupeEnabled    = true;
    public boolean dedupeUseIoU     = true;   // ← switch here

    // IoU mode params
    public double dedupeMinIoU      = 0.3;    // merge if box overlap >= 30%

    // Center-distance mode params
    public double dedupeMaxCenterDistPx = 25;

    // Shared: which detection to keep when merging
    public boolean keepHigherConfidenceOnDedupe = true;

    // ----------------------------
    // Ramp LOW point (field inches)
    // ----------------------------
    private static final double FIELD_SIZE = 144.0;
    private static final Vec3 RED_BOTTOM_NOT_EDGE  = new Vec3(137.7, 71.0, 8.6);
    private static final Vec3 BLUE_BOTTOM_NOT_EDGE = new Vec3(FIELD_SIZE - 137.7, 71.0, 8.6);

    // ----------------------------
    // Outputs
    // ----------------------------
    private final ArrayList<BallObs> onRampBalls  = new ArrayList<>();
    private final ArrayList<BallObs> offRampBalls = new ArrayList<>();

    public enum BallColor { GREEN, PURPLE, UNKNOWN }

    public List<BallObs> getOnRampBalls()        { return onRampBalls; }
    public int           getOnRampArtifactAmount() { return onRampBalls.size(); }
    public List<BallObs> getOffRampBalls()       { return offRampBalls; }

    // ----------------------------
    // Main update
    // ----------------------------
    public void update(Pose robotFieldPose,
                       LLResult ll,
                       boolean isRedAlliance,
                       Telemetry telemetry,
                       TelemetryManager telemetryM) {

        onRampBalls.clear();
        offRampBalls.clear();

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

        // 1) Compute dividerY by projecting ramp bottom point to pixel space
        Vec3  lowRamp = isRedAlliance ? RED_BOTTOM_NOT_EDGE : BLUE_BOTTOM_NOT_EDGE;
        Pixel lowPx   = projectFieldPointToPixel(robotFieldPose, lowRamp);

        if (!lowPx.valid) {
            if (telemetry != null) telemetry.addLine("[Ramp] Low ramp projection invalid (behind camera?)");
            return;
        }

        double dividerY = lowPx.y;

        if (telemetry  != null) telemetry.addData("[Ramp] dividerY", "%.1f", dividerY);
        if (telemetryM != null) telemetryM.addData("rampDividerY", dividerY);

        // 2) Gather candidates
        ArrayList<Det2D> candidates = new ArrayList<>();
        for (int i = 0; i < dets.size(); i++) {
            LLResultTypes.DetectorResult d = dets.get(i);
            int classId = d.getClassId();
            BallColor color = classIdToBallColor(classId);
            double px   = safeGetXPx(d);
            double py   = safeGetYPx(d);
            double area = safeGetArea(d);
            double conf = d.getConfidence();
            candidates.add(new Det2D(i, classId, color, conf, area, px, py));
        }

        // 3) Dedupe — switch between IoU and center-distance
        if (dedupeEnabled && candidates.size() >= 2) {
            if (dedupeUseIoU) {
                candidates = dedupeByIoU(candidates);
            } else {
                candidates = dedupeByCenterDist(candidates);
            }
        }

        // 4) Classify ramp vs off-ramp
        for (Det2D det : candidates) {
            boolean isOnRamp = rampIsAboveLine
                    ? det.py < (dividerY - dividerMarginPx)
                    : det.py > (dividerY + dividerMarginPx);

            BallObs obs = new BallObs(
                    det.color, det.px, det.py, det.conf, det.area, det.classId, det.index);

            if (isOnRamp) onRampBalls.add(obs);
            else          offRampBalls.add(obs);
        }

        // 5) Sort on-ramp balls top→bottom (descending py = closer to intake exit)
        onRampBalls.sort(Comparator.comparingDouble(o -> -o.py));

        // 6) Telemetry
        if (telemetry != null) {
            telemetry.addData("[Ramp] mode",           dedupeUseIoU ? "IoU" : "CenterDist");
            telemetry.addData("[Ramp] rawDetCount",    dets.size());
            telemetry.addData("[Ramp] candAfterDedupe", candidates.size());
            telemetry.addData("[Ramp] onRamp",         onRampBalls.size());
            telemetry.addData("[Ramp] offRamp",        offRampBalls.size());
            for (int i = 0; i < onRampBalls.size(); i++) {
                BallObs b = onRampBalls.get(i);
                telemetry.addData("[Ramp] ON #" + i,
                        "%s px=%.1f py=%.1f conf=%.2f area=%.3f",
                        b.color, b.px, b.py, b.conf, b.area);
            }
        }

        if (telemetryM != null) {
            telemetryM.addData("onRampCount",  onRampBalls.size());
            telemetryM.addData("offRampCount", offRampBalls.size());
            if (!onRampBalls.isEmpty()) {
                telemetryM.addData("onRamp0_py",    onRampBalls.get(0).py);
                telemetryM.addData("onRamp0_color", onRampBalls.get(0).color.toString());
            }
        }
    }

    // ----------------------------
    // Dedupe — IoU mode (scale-invariant)
    // Merges two detections if their approximate bounding boxes overlap by >= dedupeMinIoU.
    // Box is approximated as a square from getTargetArea() (fraction of image).
    // Works correctly regardless of robot distance to ramp.
    // ----------------------------
    private ArrayList<Det2D> dedupeByIoU(ArrayList<Det2D> in) {
        boolean[] removed = new boolean[in.size()];

        for (int i = 0; i < in.size(); i++) {
            if (removed[i]) continue;
            Det2D a = in.get(i);

            for (int j = i + 1; j < in.size(); j++) {
                if (removed[j]) continue;
                Det2D b = in.get(j);

                if (iou(a, b) < dedupeMinIoU) continue;

                int removeIdx = pickRemove(a, b, j, i);
                removed[removeIdx] = true;
                if (removeIdx == i) break;
            }
        }

        return collect(in, removed);
    }

    // IoU helper — approximate square bounding box from area fraction
    private double iou(Det2D a, Det2D b) {
        double areaA = a.area * imageW * imageH;
        double areaB = b.area * imageW * imageH;
        double sideA = Math.sqrt(areaA);
        double sideB = Math.sqrt(areaB);

        double ax1 = a.px - sideA / 2, ax2 = a.px + sideA / 2;
        double ay1 = a.py - sideA / 2, ay2 = a.py + sideA / 2;
        double bx1 = b.px - sideB / 2, bx2 = b.px + sideB / 2;
        double by1 = b.py - sideB / 2, by2 = b.py + sideB / 2;

        double interX = Math.max(0, Math.min(ax2, bx2) - Math.max(ax1, bx1));
        double interY = Math.max(0, Math.min(ay2, by2) - Math.max(ay1, by1));
        double intersection = interX * interY;

        double union = areaA + areaB - intersection;
        if (union <= 0) return 0;
        return intersection / union;
    }

    // ----------------------------
    // Dedupe — center-distance mode (simple, fixed threshold)
    // Merges two detections if their pixel centers are within dedupeMaxCenterDistPx.
    // Simple but the threshold needs re-tuning as robot distance changes.
    // ----------------------------
    private ArrayList<Det2D> dedupeByCenterDist(ArrayList<Det2D> in) {
        boolean[] removed = new boolean[in.size()];

        for (int i = 0; i < in.size(); i++) {
            if (removed[i]) continue;
            Det2D a = in.get(i);

            for (int j = i + 1; j < in.size(); j++) {
                if (removed[j]) continue;
                Det2D b = in.get(j);

                double dist = Math.hypot(a.px - b.px, a.py - b.py);
                if (dist > dedupeMaxCenterDistPx) continue;

                int removeIdx = pickRemove(a, b, j, i);
                removed[removeIdx] = true;
                if (removeIdx == i) break;
            }
        }

        return collect(in, removed);
    }

    // Shared: decide which of two overlapping detections to remove
    private int pickRemove(Det2D a, Det2D b, int idxJ, int idxI) {
        if (keepHigherConfidenceOnDedupe) {
            return (a.conf >= b.conf) ? idxJ : idxI;
        } else {
            return (a.area >= b.area) ? idxJ : idxI;
        }
    }

    // Shared: collect non-removed entries
    private ArrayList<Det2D> collect(ArrayList<Det2D> in, boolean[] removed) {
        ArrayList<Det2D> out = new ArrayList<>();
        for (int i = 0; i < in.size(); i++) {
            if (!removed[i]) out.add(in.get(i));
        }
        return out;
    }

    // ----------------------------
    // Helpers
    // ----------------------------
    private BallColor classIdToBallColor(int classId) {
        if (classId == greenBallClassId)  return BallColor.GREEN;
        if (classId == purpleBallClassId) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    // ----------------------------
    // Projection
    // ----------------------------
    private Pixel projectFieldPointToPixel(Pose robotPoseField, Vec3 fieldPoint) {
        double fx = (imageW / 2.0) / Math.tan(Math.toRadians(hfovDeg) / 2.0);
        double fy = (imageH / 2.0) / Math.tan(Math.toRadians(vfovDeg) / 2.0);
        double cx = (imageW / 2.0) + cxOffsetPx;
        double cy = (imageH / 2.0) + cyOffsetPx;

        double h    = robotPoseField.getHeading();
        double cosH = Math.cos(h);
        double sinH = Math.sin(h);

        double dxField = camForwardIn * cosH - camLeftIn * sinH;
        double dyField = camForwardIn * sinH + camLeftIn * cosH;

        Vec3 camPosField = new Vec3(
                robotPoseField.getX() + dxField,
                robotPoseField.getY() + dyField,
                camUpIn);

        Vec3 vField    = fieldPoint.minus(camPosField);
        Vec3 vRobot    = rotateAboutZ(vField, -h);
        Vec3 vRobotRot = applyInverseCamExtrinsic(vRobot);

        double xCam = -vRobotRot.x;
        double yCam = -vRobotRot.z;
        double zCam =  vRobotRot.y;

        if (zCam <= 0.5) return Pixel.invalid();

        double u = fx * (xCam / zCam) + cx;
        double v = fy * (yCam / zCam) + cy;
        return new Pixel(u, v, true);
    }

    private Vec3 applyInverseCamExtrinsic(Vec3 vRobot) {
        double yaw   = Math.toRadians(camYawDeg);
        double pitch = Math.toRadians(camPitchDeg);
        double roll  = Math.toRadians(camRollDeg);
        Vec3 r1 = rotateAboutY(vRobot, -roll);
        Vec3 r2 = rotateAboutX(r1,     -pitch);
        return    rotateAboutZ(r2,     -yaw);
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
    // Safe getters
    // ----------------------------
    private double safeGetXPx(LLResultTypes.DetectorResult d) {
        try { return d.getTargetXPixels(); } catch (Throwable ignored) {}
        return 0;
    }
    private double safeGetYPx(LLResultTypes.DetectorResult d) {
        try { return d.getTargetYPixels(); } catch (Throwable ignored) {}
        return 0;
    }
    private double safeGetArea(LLResultTypes.DetectorResult d) {
        try { return d.getTargetArea(); } catch (Throwable ignored) {}
        return 0;
    }

    // ----------------------------
    // Small structs
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

    private static class Det2D {
        final int index, classId;
        final BallColor color;
        final double conf, area, px, py;
        Det2D(int index, int classId, BallColor color, double conf, double area, double px, double py) {
            this.index = index; this.classId = classId; this.color = color;
            this.conf = conf; this.area = area; this.px = px; this.py = py;
        }
    }

    public static class BallObs {
        public final BallColor color;
        public final double px, py, conf, area;
        public final int classId, rawIndex;
        public BallObs(BallColor color, double px, double py, double conf, double area, int classId, int rawIndex) {
            this.color = color; this.px = px; this.py = py;
            this.conf = conf; this.area = area; this.classId = classId; this.rawIndex = rawIndex;
        }
    }
}