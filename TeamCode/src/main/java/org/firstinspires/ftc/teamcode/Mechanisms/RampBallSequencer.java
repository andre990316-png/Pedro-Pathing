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
 * Projects the RAMP-SIDE EDGE (bottom-not-edge -> top-not-edge) into the camera image,
 * forms a divider line in pixel space, and classifies detector results as "on ramp" vs "off ramp".
 *
 * You can tune:
 *  - camera offsets (forward/left/up, inches)
 *  - camera yaw/pitch/roll (degrees)
 *  - principal point offsets (pixels) from Limelight calibration screen if you want
 *  - rampSideIsPositive to flip which side is considered "on ramp"
 */
@Configurable
public class RampBallSequencer {

    // ----------------------------
    // Camera / image params
    // ----------------------------
    public int imageW = 640;
    public int imageH = 480;

    // From your Limelight 3A specs
    public double hfovDeg = 54.5;
    public double vfovDeg = 42.0;

    // Principal pixel offset (optional; you showed x:-2.774, y:22.549 in calibration)
    public double cxOffsetPx = -2.774;
    public double cyOffsetPx = 22.549;

    // ----------------------------
    // Camera extrinsics relative to robot (INCHES, DEGREES)
    // Robot frame assumption (matches what you said):
    //   +Y = forward, +X = left, +Z = up
    // ----------------------------
    public double camForwardIn = 8.7;   // +Y
    public double camLeftIn    = 0.0;   // +X
    public double camUpIn      = 9.55;   // +Z

    // camera rotation relative robot
    public double camYawDeg   = 0.0;    // about +Z (up)
    public double camPitchDeg = 0.0;  // about +X (left). negative = pitched "down" in this convention
    public double camRollDeg  = 0.0;    // about +Y (forward)

    // If classification is reversed, flip this
    public boolean rampSideIsPositive = true;

    // ----------------------------
    // Ramp edge points (FIELD inches)
    // You provided RED ramp "not on field edge" line:
    //   bottom: (137.7, 71, 8.6)
    //   top:    (137.7,114,19.7)
    // For BLUE we mirror in X: x' = 144 - x
    // ----------------------------
    private static final double FIELD_SIZE = 144.0;

    private static final Vec3 RED_BOTTOM_NOT_EDGE = new Vec3(137.7, 71.0, 8.6);
    private static final Vec3 RED_TOP_NOT_EDGE    = new Vec3(137.7,114.0,19.7);

    private static final Vec3 BLUE_BOTTOM_NOT_EDGE = new Vec3(FIELD_SIZE - 137.7, 71.0, 8.6);
    private static final Vec3 BLUE_TOP_NOT_EDGE    = new Vec3(FIELD_SIZE - 137.7,114.0,19.7);

    // ----------------------------
    // Public API
    // ----------------------------
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

        // 1) Build divider line by projecting the ramp side edge into pixels
        Vec3 p0 = isRedAlliance ? RED_BOTTOM_NOT_EDGE : BLUE_BOTTOM_NOT_EDGE;
        Vec3 p1 = isRedAlliance ? RED_TOP_NOT_EDGE    : BLUE_TOP_NOT_EDGE;

        Pixel a = projectFieldPointToPixel(robotFieldPose, p0, telemetry);
        Pixel b = projectFieldPointToPixel(robotFieldPose, p1, telemetry);

        if (!a.valid || !b.valid) {
            if (telemetry != null) telemetry.addLine("[Ramp] Ramp edge projection failed (behind camera?)");
            return;
        }

        // Line in image: through (a.x,a.y) and (b.x,b.y)
        Line2 line = Line2.fromTwoPoints(a.x, a.y, b.x, b.y);

        if (telemetry != null) {
            telemetry.addData("[Ramp] lineA(x,y)", "%.1f, %.1f", a.x, a.y);
            telemetry.addData("[Ramp] lineB(x,y)", "%.1f, %.1f", b.x, b.y);
        }
        if (telemetryM != null) {
            telemetryM.addData("rampLineAx", a.x);
            telemetryM.addData("rampLineAy", a.y);
            telemetryM.addData("rampLineBx", b.x);
            telemetryM.addData("rampLineBy", b.y);
        }

        // 2) Classify balls as on-ramp/off-ramp by which side of the line their pixel center lies on
        ArrayList<DetPx> onRamp = new ArrayList<>();
        ArrayList<DetPx> offRamp = new ArrayList<>();

        for (int i = 0; i < dets.size(); i++) {
            LLResultTypes.DetectorResult d = dets.get(i);

            // Many SDK versions provide these. If your autocomplete differs, swap to the correct getters.
            double px = safeGetX(d);
            double py = safeGetY(d);

            double sideVal = line.side(px, py); // >0 one side, <0 other side

            boolean isOnRamp = rampSideIsPositive ? (sideVal > 0) : (sideVal < 0);

            DetPx dp = new DetPx(i, d.getClassId(), d.getConfidence(), px, py, sideVal);

            if (isOnRamp) onRamp.add(dp);
            else offRamp.add(dp);
        }

        // 3) Sequence (you said balls flow top -> bottom). Sort on-ramp by y descending (bigger y = lower in image)
        onRamp.sort(Comparator.comparingDouble(o -> -o.py));

        // 4) Telemetry output
        if (telemetry != null) {
            telemetry.addData("[Ramp] detCount", dets.size());
            telemetry.addData("[Ramp] onRamp", onRamp.size());
            telemetry.addData("[Ramp] offRamp", offRamp.size());

            for (int i = 0; i < onRamp.size(); i++) {
                DetPx d = onRamp.get(i);
                telemetry.addData("[Ramp] ON #" + i,
                        "idx=%d class=%d conf=%.2f px=%.1f py=%.1f side=%.2f",
                        d.index, d.classId, d.conf, d.px, d.py, d.sideVal);
            }
        }

        if (telemetryM != null) {
            telemetryM.addData("onRampCount", onRamp.size());
            telemetryM.addData("offRampCount", offRamp.size());
            if (!onRamp.isEmpty()) {
                telemetryM.addData("onRampTop_px", onRamp.get(0).px);
                telemetryM.addData("onRampTop_py", onRamp.get(0).py);
            }
        }
    }

    // ----------------------------
    // Projection math
    // ----------------------------

    private Pixel projectFieldPointToPixel(Pose robotPoseField, Vec3 fieldPoint, Telemetry telemetry) {
        // Intrinsics from FOV
        double fx = (imageW / 2.0) / Math.tan(Math.toRadians(hfovDeg) / 2.0);
        double fy = (imageH / 2.0) / Math.tan(Math.toRadians(vfovDeg) / 2.0);
        double cx = (imageW / 2.0) + cxOffsetPx;
        double cy = (imageH / 2.0) + cyOffsetPx;

        // Camera position in field (inches)
        // Robot pose: (x,y,heading) where heading 0 = +X, CCW positive
        double h = robotPoseField.getHeading();
        double cosH = Math.cos(h);
        double sinH = Math.sin(h);

        // Robot frame offset -> field offset
        // robot +Y forward, +X left
        double dxField = camForwardIn * cosH - camLeftIn * sinH;
        double dyField = camForwardIn * sinH + camLeftIn * cosH;
        dxField = camForwardIn * cosH - camLeftIn * sinH;
        dyField = camForwardIn * sinH + camLeftIn * cosH;
        Vec3 camPosField = new Vec3(
                robotPoseField.getX() + dxField,
                robotPoseField.getY() + dyField,
                camUpIn
        );

        // Vector from camera to point in FIELD
        Vec3 vField = fieldPoint.minus(camPosField);

        // Rotate field vector into ROBOT frame by -heading
        Vec3 vRobot = rotateAboutZ(vField, -h);

        // Apply camera relative rotation (yaw/pitch/roll) in ROBOT frame, then map to CAMERA axes
        // Step A: rotate robot->camera mount (inverse because we want vector in camera coords)
        Vec3 vRobotRot = applyInverseCamExtrinsic(vRobot);

        // Step B: base mapping robot axes -> camera axes
        // robot: +X left, +Y forward, +Z up
        // cam:   +Z forward, +X right, +Y down
        // x_right = -x_left
        // y_down  = -z_up
        // z_fwd   = +y_fwd
        double xCam = vRobotRot.x;
        double yCam = vRobotRot.z;
        double zCam = -vRobotRot.y;

        if (telemetry != null) {
            telemetry.addData("vRobotRot", "x=%.2f y=%.2f z=%.2f", vRobotRot.x, vRobotRot.y, vRobotRot.z);
            telemetry.addData("camXYZ", "x=%.2f y=%.2f z=%.2f", xCam, yCam, zCam);
        }

        // Behind camera or too close
        if (zCam <= 0.5) return Pixel.invalid();

        // Pinhole projection
        double u = fx * (xCam / zCam) + cx;
        double v = fy * (yCam / zCam) + cy;

        return new Pixel(u, v, true);
    }

    private Vec3 applyInverseCamExtrinsic(Vec3 vRobot) {
        // Camera yaw/pitch/roll are mount rotations RELATIVE ROBOT.
        // To convert a robot-frame vector into camera-mounted-rotated robot frame, apply inverse rotations.

        double yaw = Math.toRadians(camYawDeg);
        double pitch = Math.toRadians(camPitchDeg);
        double roll = Math.toRadians(camRollDeg);

        // Inverse = apply negative angles in reverse order
        Vec3 r1 = rotateAboutY(vRobot, -roll);   // inverse roll about robot +Y
        Vec3 r2 = rotateAboutX(r1, -pitch);      // inverse pitch about robot +X
        Vec3 r3 = rotateAboutZ(r2, -yaw);        // inverse yaw about robot +Z
        return r3;
    }

    // Rotations in ROBOT axes
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
    // Detector pixel getters (SDK differences safety)
    // ----------------------------
    private double safeGetX(LLResultTypes.DetectorResult d) {
        try { return d.getTargetXPixels(); } catch (Throwable ignored) {}
        return 0;
    }
    private double safeGetY(LLResultTypes.DetectorResult d) {
        try { return d.getTargetYPixels(); } catch (Throwable ignored) {}
        return 0;
    }

    // ----------------------------
    // Small helper structs
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

    private static class Line2 {
        // ax + by + c = 0
        final double a, b, c;
        Line2(double a, double b, double c) { this.a = a; this.b = b; this.c = c; }

        static Line2 fromTwoPoints(double x1, double y1, double x2, double y2) {
            double a = (y1 - y2);
            double b = (x2 - x1);
            double c = (x1 * y2 - x2 * y1);
            return new Line2(a, b, c);
        }

        double side(double x, double y) {
            return a * x + b * y + c;
        }
    }

    private static class DetPx {
        final int index;
        final int classId;
        final double conf;
        final double px, py;
        final double sideVal;

        DetPx(int index, int classId, double conf, double px, double py, double sideVal) {
            this.index = index;
            this.classId = classId;
            this.conf = conf;
            this.px = px;
            this.py = py;
            this.sideVal = sideVal;
        }
    }
}