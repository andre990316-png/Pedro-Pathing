package org.firstinspires.ftc.teamcode.subsystem.shooter;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Config;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.configs.FlyWheelConfig;
import org.firstinspires.ftc.teamcode.utils.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.utils.configs.Team;

import org.firstinspires.ftc.teamcode.op.OpObject;
import org.firstinspires.ftc.teamcode.op.OpStruct;

@Configurable
public class Shooter extends OpStruct {
    private Shooter(){}
    private final static Shooter m_instance;
    static {
        m_instance = new Shooter();
        OpObject.addStruct(m_instance);
    }

    public static void setEnable(boolean isEnable)           { Shooter.isEnable = isEnable; }
    public static void setEnableAiming(boolean enableAiming) { Shooter.enableAiming = enableAiming; }
    public static void resetPitch() {
        Shooter.m_rpmBufferIdx = 0;
        m_rpmBufferFull = false;
        java.util.Arrays.fill(m_rpmBuffer, 0.0);
    }

    public static double  targetRPM     = 0;
    public static double  actualRPM     = 0;  // exposed so IsStable() can be checked from outside
    public static boolean isEnable      = false;
    public static boolean enableAiming  = true;
    public static boolean enablePredict = true;

    public final static ShooterPose current_position        = new ShooterPose();
    public final static ShooterPose target_position         = new ShooterPose();
    public final static ShooterPose predict_target_position = new ShooterPose();

    public static DcMotorEx m_flyWheel1, m_flyWheel2;
    private static Servo     m_pitch;
    private static Servo     m_aimX1, m_aimX2;

    // Two controllers share the same PID values from Config
    private final PIDController m_flyWheelController1 = Config.flyWheelController1;
    private final PIDController m_flyWheelController2 = Config.flyWheelController2;

    private static Follower m_follower = null;

    private double m_lastTargetVelocity = 0;
    private double m_lastTargetPitch    = 40.0;
    // Rolling average for pitch feedback smoothing
    private static final int RPM_BUFFER_SIZE = 7; // N samples to average
    private static final double[] m_rpmBuffer = new double[RPM_BUFFER_SIZE];
    private static int m_rpmBufferIdx = 0;
    private static boolean m_rpmBufferFull = false;

    // -------------------------------------------------------
    // IsStable: true when actual RPM is close enough to target
    // -------------------------------------------------------
    public static boolean IsStable() {
        if (!isEnable) return false;
        return Math.abs(targetRPM - actualRPM) < ShooterConfig.RPM_ERROR_THRESHOLD;
    }

    // -------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------
    @Override
    public void Init() {
        m_flyWheel1 = init_flyWheel(Config.fly_wheel_1); // ShooterL — no encoder
        m_flyWheel2 = init_flyWheel(Config.fly_wheel_2); // ShooterR — HAS encoder
        m_pitch     = init_servo(Config.pitch);
        m_aimX1     = init_servo(Config.aimX1);
        m_aimX2     = init_servo(Config.aimX2);
    }

    @Override
    public void Start() {
        m_follower = Drivetrain.getFollower();

        // Set target basket position based on alliance
        Pose aimTarget = (Config.team == Team.Blue)
                ? Config.maunalAimingPositionBlue
                : Config.maunalAimingPositionRed;
        target_position.x = aimTarget.getX();
        target_position.y = aimTarget.getY();
        target_position.z = ShooterConfig.H; // basket height above shooter (in)
        resetPitch();
    }

    @Override
    public void Loop() {
        // --- Update current position from odometry ---
        Pose current_pose = m_follower.getPose();
        ShooterPose rawPose = new ShooterPose(
                current_pose.getX(),
                current_pose.getY(),
                0,
                current_pose.getHeading());
        ShooterPose turretPose = currentPoseToTurretPose(rawPose);
        current_position.x   = turretPose.x;
        current_position.y   = turretPose.y;
        current_position.yaw = turretPose.yaw;


        // --- Predict target (motion compensation) ---
        // Shift aim point opposite to robot velocity × flytime.
        // flytime uses distance to the aiming target (0,130)/(144,130) —
        // where the ball actually lands, not the opening edge.
        // --- Predict target (motion compensation) ---
        if (enablePredict) {
            Vector rv = m_follower.getVelocity();
//
//            // 針對實務物理模擬調整之收斂閾值
//            final double D_THRESHOLD = 0.5;   // 英吋
//            final double T_THRESHOLD = 0.01;  // 秒
//            final int    MAX_ITER    = 10;
//
//            double cosP = Math.cos(Math.toRadians(m_lastTargetPitch));
//
//            // 避免除以零錯誤，使用極小正值防護
//            double vHoriz = (cosP > 0 && m_lastTargetVelocity > 0)
//                    ? m_lastTargetVelocity * cosP : 0.001;
//
//            // 初始距離與飛行時間估算
//            double prevD = Math.hypot(
//                    target_position.x - current_position.x,
//                    target_position.y - current_position.y);
//            double prevT = prevD / vHoriz;
//
//            double predX = target_position.x;
//            double predY = target_position.y;
//
//            // 不動點迭代
//            for (int i = 0; i < MAX_ITER; i++) {
//                // 拋射物繼承射手速度。
//                // 相對於射手，目標以 (rv * prevT) 進行反向位移
//                predX = target_position.x - rv.getXComponent() * prevT;
//                predY = target_position.y - rv.getYComponent() * prevT;
//
//                // 計算至新虛擬目標之距離與飛行時間
//                double newD = Math.hypot(predX - current_position.x, predY - current_position.y);
//                double newT = newD / vHoriz;
//
//                if (Math.abs(newD - prevD) < D_THRESHOLD && Math.abs(newT - prevT) < T_THRESHOLD) {
//                    break;
//                }
//                prevD = newD;
//                prevT = newT;
//            }
//            predict_target_position.x = predX;
//            predict_target_position.y = predY;
            double ft = flytimeVertical(m_lastTargetVelocity, m_lastTargetPitch, ShooterConfig.H);
            predict_target_position.x = target_position.x - rv.getXComponent() * ft;
            predict_target_position.y = target_position.y - rv.getYComponent() * ft;
            predict_target_position.z = target_position.z;

        } else {
            predict_target_position.x = target_position.x;
            predict_target_position.y = target_position.y;
            predict_target_position.z = target_position.z;
        }

        // --- Read actual RPM from ShooterL (has encoder) ---
        // --- Read actual RPM and push into rolling buffer ---
        double rawRPM = (m_flyWheel1 != null)
                ? ShooterCalculator.TicksPerSecToRPM(m_flyWheel1.getVelocity())
                : 0;
        m_rpmBuffer[m_rpmBufferIdx] = rawRPM;
        m_rpmBufferIdx = (m_rpmBufferIdx + 1) % RPM_BUFFER_SIZE;
        if (m_rpmBufferIdx == 0) m_rpmBufferFull = true;

        // Compute average over however many samples we have so far
        int count = m_rpmBufferFull ? RPM_BUFFER_SIZE : m_rpmBufferIdx;
        double sum = 0;
        for (int i = 0; i < count; i++) sum += m_rpmBuffer[i];
        actualRPM = (count > 0) ? sum / count : rawRPM;

        double actualVelocityInPerSec = ShooterCalculator.RPMToVelocity(actualRPM);

        // --- Stage III: Virtual target (air resistance compensation) ---
        ShooterPose virtualTarget = ShooterCalculator.calculateVirtualTarget(
                current_position, predict_target_position);

        // --- Stage IV: Ideal velocity → target RPM ---
        double targetVelocityInPerSec = ShooterCalculator.calculateIdealVelocity(
                current_position, virtualTarget, m_lastTargetVelocity);
        targetRPM = ShooterCalculator.VelocityToRPM(targetVelocityInPerSec);


        m_lastTargetVelocity = targetVelocityInPerSec;

        // --- Stage V: Pitch feedback ---
        double targetPitchDeg;
        if (IsStable()) {
            // Still spinning up — hold pitch at ideal angle to avoid servo jitter
            targetPitchDeg = ShooterCalculator.calculateRealPitchAngle(
                    current_position, virtualTarget, targetVelocityInPerSec);
        } else {
            // Flywheel on target — use real velocity for precise pitch
            if(Shooter.isEnable) {
                targetPitchDeg = ShooterCalculator.calculateRealPitchAngle(
                        current_position, virtualTarget, actualVelocityInPerSec);
            }
            else {
                targetPitchDeg = ShooterCalculator.calculateRealPitchAngle(
                        current_position, virtualTarget, targetVelocityInPerSec);
            }
        }
//        targetPitchDeg = ShooterCalculator.calculateRealPitchAngle(
//                current_position, virtualTarget, actualVelocityInPerSec);
//        m_lastTargetPitch = targetPitchDeg;

        // --- Apply to hardware ---
        update_yaw();
        update_pitch(targetPitchDeg);

        if (isEnable) {
            // Both motors get same power — only ShooterR (m_flyWheel2) has encoder
            update_flyWheel(m_flyWheel1, m_flyWheelController1, targetRPM, Config.fly_wheel_1.maxVelocity);
            update_flyWheel(m_flyWheel2, m_flyWheelController2, targetRPM, Config.fly_wheel_2.maxVelocity);
        } else {
            if (m_flyWheel1 != null) m_flyWheel1.setPower(0);
            if (m_flyWheel2 != null) m_flyWheel2.setPower(0);
        }



        // --- Debug ---
//        m_object.telemetry.addData("isEnable     = ", isEnable);
//        m_object.telemetry.addData("IsStable     = ", IsStable());
//        m_object.telemetry.addData("targetRPM    = ", targetRPM);
//        m_object.telemetry.addData("actualRPM    = ", actualRPM);
//        m_object.telemetry.addData("pitchDeg     = ", targetPitchDeg);
//        m_object.telemetry.addData("yawDeg       = ", ShooterCalculator.calculateYaw(current_position, predict_target_position));
//        m_object.telemetry.addData("target       = ", target_position.x);
//        m_object.telemetry.addData("target       = ", target_position.y);
//        m_object.telemetry.addData("target       = ", target_position.z);
//        m_object.telemetry.addData("Pose Heading", Math.toDegrees(current_pose.getHeading()));
//        m_object.telemetry.addData("Pose x", current_pose.getX());
//        m_object.telemetry.addData("Pose y", current_pose.getY());
//        m_object.telemetry.addData("pitchDeg (pre-clamp)",
//                ShooterCalculator.calculateRealPitchAngle(
//                        current_position, virtualTarget, actualVelocityInPerSec));
//        m_object.telemetry.addData("pitchServoPos",
//                -0.0356895 * targetPitchDeg + 1.84294);
    }

    @Override
    public void Stop() {
        if (m_flyWheel1 != null) m_flyWheel1.setPower(0);
        if (m_flyWheel2 != null) m_flyWheel2.setPower(0);
        m_aimX1.setPosition(0.5);
        m_aimX2.setPosition(0.5);
        m_pitch.setPosition(0.825);
        m_flyWheel1 = null;
        m_flyWheel2 = null;
        m_aimX1     = null;
        m_aimX2     = null;
        m_pitch     = null;
        m_follower  = null;
    }

    // -------------------------------------------------------
    // Hardware helpers
    // -------------------------------------------------------
    private DcMotorEx init_flyWheel(FlyWheelConfig config) {
        DcMotorEx ret = m_object.hardwareMap.get(DcMotorEx.class, config.id);
        ret.setDirection(config.direction);
        ret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ret.setPower(0);
        return ret;
    }

    private Servo init_servo(ServoConfig config) {
        Servo ret = m_object.hardwareMap.get(Servo.class, config.id);
        ret.setDirection(config.direction);
        return ret;
    }

    // -------------------------------------------------------
    // Flywheel PID
    // Always reads encoder from m_flyWheel2 (ShooterR — only one with encoder)
    // Both motors receive the same calculated power output
    // -------------------------------------------------------
    private void update_flyWheel(DcMotorEx motor, PIDController controller, double targetRPM, double maxRPM) {
        final double targetPower  = targetRPM / maxRPM;
        final double currentRPM   = ShooterCalculator.TicksPerSecToRPM(m_flyWheel1.getVelocity());
        final double currentPower = currentRPM / maxRPM;
        final double newPower     = clamp(
                controller.calculate(currentPower, targetPower) + targetPower, -1.0, 1.0);
        motor.setPower(newPower);
    }

    // -------------------------------------------------------
    // Pitch servo
    // Linear fit from physical measurement: position = -0.0356895 * degree + 1.84294
    // -------------------------------------------------------
    private static void update_pitch(double targetPitchDeg) {
        if (m_pitch == null) return;
        if (Double.isNaN(targetPitchDeg) || Double.isInfinite(targetPitchDeg)) return;
        double position = clamp(-0.0383333 * targetPitchDeg + 2.15, 0.08, 1);
        m_pitch.setPosition(position);
    }

    // -------------------------------------------------------
    // Yaw servos
    // Servo mapping (physically measured):
    //   0.500 = center (robot forward)
    //   0.895 = max right  (+100°)
    //   0.105 = max left   (-100°)
    //   deg2position = (0.895 - 0.5) / 100.0 = 0.00395 per degree
    // -------------------------------------------------------
    private static void update_yaw() {
        final double target_deg   = ShooterCalculator.calculateYaw(current_position, predict_target_position);
        final double center       = 0.5;
        final double deg2position = (1 - 0.5) / 177.5; // 0.00395 per degree
        double targetPosition = center;
        if (Double.isNaN(target_deg) || Double.isInfinite(target_deg)) return;
        if (enableAiming) {
            targetPosition = clamp(center + target_deg * deg2position, 0, 1);
        }
        if (m_aimX1 != null && m_aimX2 != null) {
            m_aimX1.setPosition(targetPosition);
            m_aimX2.setPosition(targetPosition);
        }
    }

    private double flytimeVertical(double v, double pitchDeg, double h) {
        if (v <= 0) return 0;
        double g = ShooterConfig.G;
        double vz = v * Math.cos(Math.toRadians(pitchDeg)); // servo convention: cos not sin
        double disc = vz * vz - 2.0 * g * h;
        if (disc < 0) return 0;
        double sqrt = Math.sqrt(disc);
        double tA = (vz + sqrt) / g;
        double tB = (vz - sqrt) / g;
        if (tA > 0 && tB > 0) return Math.max(tA, tB);
        if (tA > 0) return tA;
        if (tB > 0) return tB;
        return 0;
    }
    /**
     * Converts robot center pose (from odometry) to the turret's actual
     * field position, accounting for its physical mount offset.
     *
     * Robot frame: +Y = forward, +X = left
     * Applies a 2D rotation of the offset vector by the robot heading.
     */
    private ShooterPose currentPoseToTurretPose(ShooterPose currentPose) {
        double heading = currentPose.yaw;
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double fwd  = ShooterConfig.TURRET_FORWARD_IN;
        double left = ShooterConfig.TURRET_LEFT_IN;

        // Rotate offset from robot frame into field frame
        // field_x += fwd*cos(h) - left*sin(h)
        // field_y += fwd*sin(h) + left*cos(h)
        double turretX = currentPose.x + fwd * cosH - left * sinH;
        double turretY = currentPose.y + fwd * sinH + left * cosH;

        return new ShooterPose(turretX, turretY, currentPose.z, currentPose.yaw);
    }
}