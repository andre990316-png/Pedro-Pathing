package org.firstinspires.ftc.teamcode.subsystem.shooter;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import org.firstinspires.ftc.teamcode.utils.Config;
import org.firstinspires.ftc.teamcode.utils.configs.ShooterConfig;

public class ShooterCalculator {

    /**
     * 從 Encoder Ticks 每秒轉換為 RPM
     */
    public static double TicksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / (ShooterConfig.TICKS_PER_REVOLUTION * ShooterConfig.GEAR_RATIO)) * 60.0;
    }

    /**
     * 階段 I：物理層轉換 (RPM → 初速)
     * 將馬達 RPM 轉換為球體出膛的真實線速度 (in/sec)。
     * 注意：針對「單飛輪 + 弧形軌道 (Hooded Shooter)」的物理特性，
     * 球體的理論初速只有飛輪表面線速度的「一半」。
     * 最後乘上動態擬合常數 ENERGY_CONVERSION_RATE 來彌補打滑與壓縮損耗。
     */
    public static double RPMToVelocity(double rpm) {
        double rps = rpm / 60.0;
        double surfaceVelocity = rps * (2.0 * Math.PI * ShooterConfig.FLYWHEEL_RADIUS_IN);
        double theoreticalExitVelocity = surfaceVelocity / 2.0;
        return theoreticalExitVelocity * ShooterConfig.ENERGY_CONVERSION_RATE + ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC;
    }

    /**
     * 階段 I：物理層轉換 (初速 → RPM)
     * 將數學模型算出的目標線速度 (in/sec) 轉換回目標 RPM 供 PID 使用。
     */
    public static double VelocityToRPM(double velocity) {
        double theoreticalExitVelocity = (velocity - ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC) / ShooterConfig.ENERGY_CONVERSION_RATE;
        double requiredSurfaceVelocity = theoreticalExitVelocity * 2.0;
        double rps = requiredSurfaceVelocity / (2.0 * Math.PI * ShooterConfig.FLYWHEEL_RADIUS_IN);
        return rps * 60.0;
    }

    /**
     * SOP 步驟 2：近距離測試 (求解 ENERGY_CONVERSION_RATE)
     * 在近距離 (例如 24 吋，忽略空氣阻力) 固定仰角為 45 度發射，
     * 調整至完美命中後，傳入該完美 RPM，此函式會回傳傳輸效率。
     *
     * @param distance_in  離目標的實際水平距離 (in)
     * @param height_in    與目標的垂直高度差 (in)
     * @param perfect_rpm  能在此距離完美命中目標的飛輪轉速
     * @return 該寫入 ShooterConfig.ENERGY_CONVERSION_RATE 的新數值
     */
    public static double calibrateEnergyConversionRate(double distance_in, double height_in, double perfect_rpm) {
        double theta_rad = Math.toRadians(45.0);
        double tan_theta = Math.tan(theta_rad);
        double cos_theta = Math.cos(theta_rad);

        double denominator = 2.0 * Math.pow(cos_theta, 2) * (distance_in * tan_theta - height_in);
        if (denominator <= 0) {
            return 1.0;
        }
        double ideal_velocity = Math.sqrt((ShooterConfig.G * Math.pow(distance_in, 2)) / denominator);

        double rps = perfect_rpm / 60.0;
        double surfaceVelocity = rps * (2.0 * Math.PI * ShooterConfig.FLYWHEEL_RADIUS_IN);
        double theoreticalExitVelocity = surfaceVelocity / 2.0;

        return ideal_velocity / theoreticalExitVelocity;
    }

    /**
     * SOP 步驟 3：遠距離測試 (求解 ENERGY_LOSS_RATE)
     * 確保已填入正確的 ENERGY_CONVERSION_RATE 後，退至遠距離測試。
     * 若球提早下墜，逐步調大 ENERGY_LOSS_RATE (例如每次 +0.001)。
     */

    /**
     * 階段 III：虛擬座標變換 (數學模型層)
     * 透過 ENERGY_LOSS_RATE 產生一個「虛擬目標點 B'」，補償空氣阻力與 Magnus 效應。
     */
    public static ShooterPose calculateVirtualTarget(ShooterPose current, ShooterPose realTarget) {
        double d = Math.hypot(realTarget.x - current.x, realTarget.y - current.y);

        double virtual_d = d * (1.0 + ShooterConfig.ENERGY_LOSS_RATE * d);
        double virtual_h = (realTarget.z - current.z) + (ShooterConfig.ENERGY_LOSS_RATE * Math.pow(d, 2));

        double yaw_rad = Math.atan2(realTarget.y - current.y, realTarget.x - current.x);
        double virtual_x = current.x + virtual_d * Math.cos(yaw_rad);
        double virtual_y = current.y + virtual_d * Math.sin(yaw_rad);
        double virtual_z = current.z + virtual_h;

        return new ShooterPose(virtual_x, virtual_y, virtual_z, realTarget.yaw);
    }

    /**
     * 階段 IV：計算理想初速 (前饋 Feedforward)
     *
     * CONVENTION NOTE:
     *   Servo convention:   0° = vertical, 90° = horizontal  (MIN_PITCH=30°, MAX_PITCH=54°)
     *   Physics convention: 0° = horizontal, 90° = vertical
     *   Relationship:  physics_angle = 90° - servo_angle
     *
     * All projectile math is done in physics convention.
     * calculateRealPitchAngle converts the result back to servo convention for the hardware.
     */
    public static double calculateIdealVelocity(ShooterPose current, ShooterPose virtualTarget, double lastVelocity) {
        double d = Math.hypot(virtualTarget.x - current.x, virtualTarget.y - current.y);
        double h = virtualTarget.z - current.z;
        if (d < 1.0) return lastVelocity;

        // Convert servo limits to physics angles (from horizontal):
        //   MIN_PITCH_SERVO (30°) → physics 60° (steepest physical shot)
        //   MAX_PITCH_SERVO (54°) → physics 36° (flattest physical shot)
        double physics_steepest = 90.0 - ShooterConfig.MIN_PITCH_ANGLE_DEG; // 60°
        double physics_flattest  = 90.0 - ShooterConfig.MAX_PITCH_ANGLE_DEG; // 36°
//        physics_steepest = ShooterConfig.MIN_PITCH_ANGLE_DEG; // 60°
//        physics_flattest  = ShooterConfig.MAX_PITCH_ANGLE_DEG; // 36°

        // d_cross: distance where incidentToLaunch(IDEAL_INCIDENT) == physics_steepest
        //   → beyond d_cross the ideal incident angle is achievable within servo range
        // d_min: distance where physics_steepest barely reaches h
        //   → closer than d_min the servo physically cannot arc the ball over the lip
        double d_cross = 2.0 * h / (Math.tan(Math.toRadians(physics_steepest))
                - Math.tan(Math.toRadians(ShooterConfig.IDEAL_INCIDENT_ANGLE_DEG)));
        double d_min   = h / Math.tan(Math.toRadians(physics_steepest));

        // Select launch angle in PHYSICS convention (from horizontal)
        double launchPhysicsDeg;
        if (d >= d_cross) {
            // Far range: exact ideal incident angle — servo is within range
            launchPhysicsDeg = incidentToLaunchAngle(
                    ShooterConfig.IDEAL_INCIDENT_ANGLE_DEG, d, h);
        } else if (d <= d_min) {
            // Too close — servo at steepest limit, ascending arc unavoidable
            launchPhysicsDeg = physics_steepest;
        } else {
            // Mid range: blend steepest → flattest as distance increases
            double t = (d - d_min) / (d_cross - d_min);
            launchPhysicsDeg = physics_steepest - t * (physics_steepest - physics_flattest);
        }

        // Clamp in physics space
        launchPhysicsDeg = clamp(launchPhysicsDeg, physics_flattest, physics_steepest);

        double theta_rad   = Math.toRadians(launchPhysicsDeg);
        double tan_theta   = Math.tan(theta_rad);
        double cos_theta   = Math.cos(theta_rad);
        double denominator = 2.0 * Math.pow(cos_theta, 2) * (d * tan_theta - h);

        double v_ideal;
        if (denominator > 0) {
            v_ideal = Math.sqrt((ShooterConfig.G * Math.pow(d, 2)) / denominator);
        } else {
            v_ideal = lastVelocity;
        }
        v_ideal = Math.min(v_ideal, ShooterConfig.MAX_VELOCITY_IN_PER_SEC);

        // ── Front wall clearance check ────────────────────────────────────────
        // Goal corner: red at (144,144), blue at (0,144).
        // Front wall is 18.3/sqrt(2) in along the 45° diagonal from corner.
        // Ball center must clear the front lip by at least FRONT_WALL_CLEARANCE_IN
        // (set to ball radius 2.5 in + safety margin = 3.0 in in Config).
        double d_fw = frontWallDistance(current, virtualTarget);
        if (d_fw > 0 && d_fw < d) {
            double h_at_fw = ballHeightAt(d_fw, v_ideal, launchPhysicsDeg);
            if (h_at_fw < h + ShooterConfig.FRONT_WALL_CLEARANCE_IN) {
                // Ball clips wall — step physics angle steeper 1° at a time until clearance met
                double physicsAngle = launchPhysicsDeg;
                for (int i = 0; i < 20 && physicsAngle < physics_steepest; i++) {
                    physicsAngle = Math.min(physicsAngle + 1.0, physics_steepest);
                    double theta2  = Math.toRadians(physicsAngle);
                    double denom2  = 2.0 * Math.pow(Math.cos(theta2), 2) * (d * Math.tan(theta2) - h);
                    if (denom2 <= 0) continue;
                    double v2 = Math.sqrt(ShooterConfig.G * d * d / denom2);
                    if (ballHeightAt(d_fw, v2, physicsAngle) >= h + ShooterConfig.FRONT_WALL_CLEARANCE_IN) {
                        v_ideal = Math.min(v2, ShooterConfig.MAX_VELOCITY_IN_PER_SEC);
                        break;
                    }
                }
            }
        }

        return v_ideal;
    }

    /**
     * 將理想入射角 (arrival angle) 轉換為發射仰角 (launch angle)。
     * 兩者均以「物理慣例」表示（從水平面量測，0°=平射，90°=垂直）。
     *
     *   tan(θ_launch) = tan(θ_incident) + 2h/d
     *
     * @param incidentAngleDeg  入射角，物理慣例 (°)
     * @param d                 水平距離 (in)
     * @param h                 高度差 (in)
     * @return 對應的發射仰角，物理慣例 (°)
     */
    public static double incidentToLaunchAngle(double incidentAngleDeg, double d, double h) {
        if (d <= 0) return incidentAngleDeg;
        double tan_incident = Math.tan(Math.toRadians(incidentAngleDeg));
        double tan_launch   = tan_incident + (2.0 * h / d);
        return Math.toDegrees(Math.atan(tan_launch));
    }

    /**
     * 階段 V：計算實際仰角 (反饋 Feedback)
     * 根據飛輪真實轉速，反解拋物線二次方程式，求出能打中虛擬目標的實際仰角。
     *
     * Returns SERVO angle (servo convention: 0=vertical, 90=horizontal).
     * The quadratic is solved in physics convention; result is converted before return.
     */
    public static double calculateRealPitchAngle(ShooterPose current, ShooterPose virtualTarget, double actual_velocity) {
        if (actual_velocity <= 0) {
            return ShooterConfig.MAX_PITCH_ANGLE_DEG;
        }

        double d = Math.hypot(virtualTarget.x - current.x, virtualTarget.y - current.y);
        double h = virtualTarget.z - current.z;

        if (d < 1.0) return ShooterConfig.MAX_PITCH_ANGLE_DEG;

        // Solve: A·tan²θ - B·tanθ + C = 0   (θ in PHYSICS convention, from horizontal)
        // A = G·d² / (2·V²),  B = d,  C = h + A
        double A = (ShooterConfig.G * Math.pow(d, 2)) / (2.0 * Math.pow(actual_velocity, 2));
        double B = d;
        double C = h + A;

        double discriminant = B * B - 4.0 * A * C;

        if (discriminant < 0) {
            // Velocity too low to reach target — return steepest servo angle
            return ShooterConfig.MAX_PITCH_ANGLE_DEG;
        }

        // Two physics solutions:
        //   theta_low  = smaller physics angle → flatter shot → ascending arc
        //   theta_high = larger  physics angle → steeper shot → descending arc
        //
        // theta_low (physics) always matches what calculateIdealVelocity computed —
        // that function works in physics space and uses the same projectile formula.
        double tan_theta_low = (B - Math.sqrt(discriminant)) / (2.0 * A);
        double theta_low_physics = Math.toDegrees(Math.atan(tan_theta_low));

        // Physics bounds corresponding to servo [MIN_PITCH, MAX_PITCH]
        double physics_flattest  = 90.0 - ShooterConfig.MAX_PITCH_ANGLE_DEG; // 36°
        double physics_steepest  = 90.0 - ShooterConfig.MIN_PITCH_ANGLE_DEG; // 60°

        // Clamp physics angle, then convert to servo convention: servo = 90 - physics
        double selected_physics = clamp(theta_low_physics, physics_flattest, physics_steepest);
        double selected_servo   = 90.0 - selected_physics;

        // Apply manual offset and clamp to servo hardware range
        selected_servo += ShooterConfig.PITCH_OFFSET_DEG;
        return clamp(selected_servo,
                ShooterConfig.MIN_PITCH_ANGLE_DEG,
                ShooterConfig.MAX_PITCH_ANGLE_DEG);
    }

    /**
     * 偏航角 (Yaw) 計算
     * 根據當前位置與目標位置，計算所需的偏航角度 (°)，正值向右。
     */
    public static double calculateYaw(ShooterPose current, ShooterPose target) {
        double targetYaw_rad = Math.atan2(target.y - current.y, target.x - current.x);
        double heading_rad   = current.yaw;
        double target_deg    = Math.toDegrees(targetYaw_rad - heading_rad);
        while (target_deg >  180) target_deg -= 360;
        while (target_deg < -180) target_deg += 360;
        return clamp(target_deg, -177.5, 177.5);
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * Returns the horizontal distance from the robot to the goal's front wall
     * along the robot→target trajectory, in inches.
     * Returns -1 if the front wall is not between robot and target.
     *
     * Front wall geometry (field top-view):
     *   The goal sits at a corner. The opening faces the field at 45°.
     *   The front wall is 18.3/sqrt(2) in along each axis from the corner.
     *
     *   Red  goal corner (144, 144): front wall line  x + y = 262.12
     *   Blue goal corner (  0, 144): front wall line  y - x = 118.12
     */
    private static double frontWallDistance(ShooterPose current, ShooterPose target) {
        final double FW_OFFSET = 18.3 / Math.sqrt(2.0); // ≈ 12.94 in
        double rx = current.x, ry = current.y;
        double tx = target.x,  ty = target.y;

        double s;
        if (tx > 72) {
            // Red goal
            double fwConst = 2.0 * (144.0 - FW_OFFSET);
            double denom   = (tx - rx) + (ty - ry);
            if (Math.abs(denom) < 1e-9) return -1;
            s = (fwConst - rx - ry) / denom;
        } else {
            // Blue goal
            double fwConst = (144.0 - FW_OFFSET) - FW_OFFSET;
            double denom   = (ty - ry) - (tx - rx);
            if (Math.abs(denom) < 1e-9) return -1;
            s = (fwConst - (ry - rx)) / denom;
        }

        if (s <= 0 || s >= 1) return -1;
        return s * Math.hypot(tx - rx, ty - ry);
    }

    /**
     * Ball height (in) at horizontal distance x from the shooter,
     * given launch speed v (in/s) and launch angle in PHYSICS convention (from horizontal).
     */
    private static double ballHeightAt(double x, double v, double physicsAngleDeg) {
        double theta = Math.toRadians(physicsAngleDeg);
        return x * Math.tan(theta)
                - ShooterConfig.G * x * x
                / (2.0 * v * v * Math.pow(Math.cos(theta), 2));
    }
}