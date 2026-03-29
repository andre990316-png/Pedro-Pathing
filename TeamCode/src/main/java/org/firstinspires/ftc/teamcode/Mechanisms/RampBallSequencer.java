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
    // Image + intrinsics
    // ----------------------------
    public int imageW = 640;
    public int imageH = 480;

    public double hfovDeg = 54.5;
    public double vfovDeg = 42.0;

    // Principal point offsets (px) from Limelight calibration (optional)
    public double cxOffsetPx = 0.0;
    public double cyOffsetPx = 0.0;

    // ----------------------------
    // Camera extrinsics relative robot (INCHES, DEGREES)
    // Your stated convention:
    // +Y = forward, +X = left, +Z = up
    // ----------------------------
    public double camForwardIn = 8.7;  // +Y
    public double camLeftIn    = 0.0;  // +X
    public double camUpIn      = 9.55; // +Z

    // camera rotation relative robot
    public double camYawDeg   = 0.0;   // about +Z (up)
    public double camPitchDeg = 0.0;   // about +X (left)
    public double camRollDeg  = 0.0;   // about +Y (forward)

    // ----------------------------
    // Horizontal divider behavior
    // ----------------------------
    // If true: py < dividerY => ON ramp (above line)
    // If false: py > dividerY => ON ramp (below line)
    public boolean rampIsAboveLine = true;

    // Add cushion margin (px) to avoid jitter near boundary
    public double dividerMarginPx = 0.0;

    // ----------------------------
    // Alliance-specific ramp filtering
    // You said: only detect BLUE ramp on blue team, RED ramp on red team.
    // Set these to your model's class IDs.
    // ----------------------------
    public boolean filterRampByAlliance = true;
    public int blueRampClassId = 0;   // TODO: set to your NN classId for "blue ramp"
    public int redRampClassId  = 1;   // TODO: set to your NN classId for "red ramp"

    // ----------------------------
    // Ball color class IDs (what you REALLY want to store)
    // Set these to your model's class IDs for the balls.
    // ----------------------------
    public int greenBallClassId  = 2; // TODO: set to your NN classId for green ball
    public int purpleBallClassId = 3; // TODO: set to your NN classId for purple ball

    // If true: ignore any detection that isn't purple/green (after ramp filtering)
    public boolean ignoreNonBallClasses = true;

    // ----------------------------
    // Duplicate suppression
    // If two detections are very close AND both have high area, treat as duplicates.
    // ----------------------------
    public boolean dedupeEnabled = true;
    public double dedupeMaxCenterDistPx = 18.0;  // distance threshold between centers
    public double dedupeAreaHigh = 0.08;         // area threshold (tune for your pipeline)
    public double dedupeAreaRatioMax = 1.6;      // (bigger/smaller) max ratio to still merge
    public boolean keepHigherConfidenceOnDedupe = true;

    // ----------------------------
    // Ramp LOW point (field inches)
    // bottom-not-on-edge point from your ramp data
    // ----------------------------
    private static final double FIELD_SIZE = 144.0;

    private static final Vec3 RED_BOTTOM_NOT_EDGE  = new Vec3(137.7, 71.0, 8.6);
    private static final Vec3 BLUE_BOTTOM_NOT_EDGE = new Vec3(FIELD_SIZE - 137.7, 71.0, 8.6);

    // ----------------------------
    // Outputs you can read from TeleOp
    // ----------------------------
    private final ArrayList<BallObs> onRampBalls = new ArrayList<>();
    private final ArrayList<BallObs> offRampBalls = new ArrayList<>();

    public enum BallColor { GREEN, PURPLE, UNKNOWN }

    /** Returns latest on-ramp sequence, sorted top->bottom (fall direction). */
    public List<BallObs> getOnRampBalls() { return onRampBalls; }

    /** Returns latest off-ramp detections (no particular ordering unless you want one). */
    public List<BallObs> getOffRampBalls() { return offRampBalls; }

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

        // 1) DividerY from low ramp point projection
        Vec3 lowRamp = isRedAlliance ? RED_BOTTOM_NOT_EDGE : BLUE_BOTTOM_NOT_EDGE;
        Pixel lowPx = projectFieldPointToPixel(robotFieldPose, lowRamp);

        if (!lowPx.valid) {
            if (telemetry != null) telemetry.addLine("[Ramp] Low ramp projection invalid (behind camera?)");
            return;
        }

        double dividerY = lowPx.y;

        if (telemetry != null) {
            telemetry.addData("[Ramp] dividerY", "%.1f", dividerY);
        }
        if (telemetryM != null) {
            telemetryM.addData("rampDividerY", dividerY);
        }

        // 2) Gather candidate detections (alliance ramp filter + optional ball-only filter)
        ArrayList<Det2D> candidates = new ArrayList<>();

        int allowedRampClass = isRedAlliance ? redRampClassId : blueRampClassId;

        for (int i = 0; i < dets.size(); i++) {
            LLResultTypes.DetectorResult d = dets.get(i);
            int classId = d.getClassId();

            if (filterRampByAlliance && classId != allowedRampClass) continue;

            BallColor color = classIdToBallColor(classId);
            if (ignoreNonBallClasses && color == BallColor.UNKNOWN) continue;

            double px = safeGetXPx(d);
            double py = safeGetYPx(d);
            double area = safeGetArea(d);
            double conf = d.getConfidence();

            candidates.add(new Det2D(i, classId, color, conf, area, px, py));
        }

        // 3) De-dupe
        if (dedupeEnabled && candidates.size() >= 2) {
            candidates = dedupe(candidates);
        }

        // 4) Classify ramp vs off-ramp using divider
        for (Det2D det : candidates) {
            boolean isOnRamp;
            if (rampIsAboveLine) {
                isOnRamp = det.py < (dividerY - dividerMarginPx);
            } else {
                isOnRamp = det.py > (dividerY + dividerMarginPx);
            }

            BallObs obs = new BallObs(det.color, det.px, det.py, det.conf, det.area, det.classId, det.index);

            if (isOnRamp) onRampBalls.add(obs);
            else offRampBalls.add(obs);
        }

        // 5) Sequence ordering: you said balls flow top->bottom.
        // In image coords, "lower" is larger py. If balls fall downward, sort DESC by py.
        onRampBalls.sort(Comparator.comparingDouble(o -> -o.py));

        // 6) Telemetry
        if (telemetry != null) {
            telemetry.addData("[Ramp] rawDetCount", dets.size());
            telemetry.addData("[Ramp] candAfterFilter", candidates.size());
            telemetry.addData("[Ramp] onRamp", onRampBalls.size());
            telemetry.addData("[Ramp] offRamp", offRampBalls.size());

            for (int i = 0; i < onRampBalls.size(); i++) {
                BallObs b = onRampBalls.get(i);
                telemetry.addData("[Ramp] ON #" + i,
                        "%s px=%.1f py=%.1f conf=%.2f area=%.3f",
                        b.color, b.px, b.py, b.conf, b.area);
            }
        }

        if (telemetryM != null) {
            telemetryM.addData("onRampCount", onRampBalls.size());
            telemetryM.addData("offRampCount", offRampBalls.size());
            if (!onRampBalls.isEmpty()) {
                telemetryM.addData("onRamp0_py", onRampBalls.get(0).py);
                telemetryM.addData("onRamp0_color", onRampBalls.get(0).color.toString());
            }
        }
    }

    // ----------------------------
    // Helpers
    // ----------------------------

    private BallColor classIdToBallColor(int classId) {
        if (classId == greenBallClassId) return BallColor.GREEN;
        if (classId == purpleBallClassId) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    private ArrayList<Det2D> dedupe(ArrayList<Det2D> in) {
        // O(n^2) is fine for small detection counts
        boolean[] removed = new boolean[in.size()];

        for (int i = 0; i < in.size(); i++) {
            if (removed[i]) continue;
            Det2D a = in.get(i);

            for (int j = i + 1; j < in.size(); j++) {
                if (removed[j]) continue;
                Det2D b = in.get(j);

                // only consider dupes if same BALL COLOR (otherwise don't merge)
                if (a.color != b.color) continue;

                double dx = a.px - b.px;
                double dy = a.py - b.py;
                double dist = Math.hypot(dx, dy);

                if (dist > dedupeMaxCenterDistPx) continue;

                // require both fairly large area to avoid merging small far objects
                if (!(a.area >= dedupeAreaHigh && b.area >= dedupeAreaHigh)) continue;

                double bigger = Math.max(a.area, b.area);
                double smaller = Math.max(1e-6, Math.min(a.area, b.area));
                double ratio = bigger / smaller;

                if (ratio > dedupeAreaRatioMax) continue;

                // decide which to remove
                int removeIdx;
                if (keepHigherConfidenceOnDedupe) {
                    removeIdx = (a.conf >= b.conf) ? j : i;
                } else {
                    // keep larger area by default
                    removeIdx = (a.area >= b.area) ? j : i;
                }

                removed[removeIdx] = true;

                // if we removed i, stop comparing it
                if (removeIdx == i) break;
            }
        }

        ArrayList<Det2D> out = new ArrayList<>();
        for (int i = 0; i < in.size(); i++) {
            if (!removed[i]) out.add(in.get(i));
        }
        return out;
    }

    // ----------------------------
    // Projection
    // ----------------------------
    private Pixel projectFieldPointToPixel(Pose robotPoseField, Vec3 fieldPoint) {
        double fx = (imageW / 2.0) / Math.tan(Math.toRadians(hfovDeg) / 2.0);
        double fy = (imageH / 2.0) / Math.tan(Math.toRadians(vfovDeg) / 2.0);
        double cx = (imageW / 2.0) + cxOffsetPx;
        double cy = (imageH / 2.0) + cyOffsetPx;

        // Robot pose: (x,y,heading) where heading 0 = +X, CCW positive
        double h = robotPoseField.getHeading();
        double cosH = Math.cos(h);
        double sinH = Math.sin(h);

        // robot +Y forward, +X left
        double dxField = camForwardIn * cosH - camLeftIn * sinH;
        double dyField = camForwardIn * sinH + camLeftIn * cosH;

        Vec3 camPosField = new Vec3(
                robotPoseField.getX() + dxField,
                robotPoseField.getY() + dyField,
                camUpIn
        );

        Vec3 vField = fieldPoint.minus(camPosField);

        // Rotate field vector into ROBOT frame by -heading
        Vec3 vRobot = rotateAboutZ(vField, -h);

        // Apply inverse mount rotation
        Vec3 vRobotRot = applyInverseCamExtrinsic(vRobot);

        // ROBOT -> CAMERA mapping
        // robot: +X left, +Y forward, +Z up
        // cam:   +Z forward, +X right, +Y down
        double xCam = -vRobotRot.x;  // right
        double yCam = -vRobotRot.z;  // down
        double zCam =  vRobotRot.y;  // forward

        if (zCam <= 0.5) return Pixel.invalid();

        double u = fx * (xCam / zCam) + cx;
        double v = fy * (yCam / zCam) + cy;

        return new Pixel(u, v, true);
    }

    private Vec3 applyInverseCamExtrinsic(Vec3 vRobot) {
        double yaw   = Math.toRadians(camYawDeg);
        double pitch = Math.toRadians(camPitchDeg);
        double roll  = Math.toRadians(camRollDeg);

        // inverse = apply negative angles in reverse order
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
    // Safe getters (SDK differences)
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
        final int index;
        final int classId;
        final BallColor color;
        final double conf;
        final double area;
        final double px, py;

        Det2D(int index, int classId, BallColor color, double conf, double area, double px, double py) {
            this.index = index;
            this.classId = classId;
            this.color = color;
            this.conf = conf;
            this.area = area;
            this.px = px;
            this.py = py;
        }
    }

    public static class BallObs {
        public final BallColor color;
        public final double px, py;
        public final double conf;
        public final double area;
        public final int classId;
        public final int rawIndex;

        public BallObs(BallColor color, double px, double py, double conf, double area, int classId, int rawIndex) {
            this.color = color;
            this.px = px;
            this.py = py;
            this.conf = conf;
            this.area = area;
            this.classId = classId;
            this.rawIndex = rawIndex;
        }
    }
}