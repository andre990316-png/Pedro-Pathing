package org.firstinspires.ftc.teamcode.utils.configs;

public class ShooterConfig {
    // 物理常數 (Physical Constants)
    public static final double G = 386.09; // 重力加速度 (in/s^2)
    public static final double H = 43.75 - 14.25;
    // Ball diameter = 5 in → radius = 2.5 in. Front wall clearance = radius + 0.5 in safety margin.
    public static double FRONT_WALL_CLEARANCE_IN = 3.0;
    public static final double MAX_VELOCITY_IN_PER_SEC = 512.0; // 飛輪最大線速度 (in/s)，約略值
    public static final double MIN_PITCH_ANGLE_DEG = 30.0; // 最小仰角硬體限制 (度)
    public static final double MAX_PITCH_ANGLE_DEG = 54.0; // 最大仰角硬體限制 (度)
    public static final double FLYWHEEL_RADIUS_IN = 2.0; // 飛輪半徑，用來計算線速度 (吋)

    // ==========================================
    // 絕對硬體常數 (Absolute Hardware Constants)
    // 這些數值取決於物理與硬體規格，通常不需要在場地微調：
    // 1. TICKS_PER_REVOLUTION: 直接查閱馬達規格表 (例如 GoBilda 裸馬達 = 28，REV HD Hex = 28)
    // 2. GEAR_RATIO: 馬達軸到飛輪軸的齒輪/皮帶傳動比 (直驅 = 1.0)
    // 3. FLYWHEEL_RADIUS_IN: 使用游標卡尺量測飛輪的實際半徑 (英吋)
    // ==========================================
    public static final double TICKS_PER_REVOLUTION = 28.0;
    public static final double GEAR_RATIO = 1.0;

    // ==========================================
    // 雙參數架構 - 場地動態擬合常數 (Two-Parameter Architecture)
    // 系統只需這兩個參數即可擬合所有的非理想物理因素 (空氣阻力、Magnus 效應、打滑損耗)：
    // 1. ENERGY_CONVERSION_RATE (能量轉換率): 近點無阻力測試求出，代表飛輪轉速轉換為球體初速的效率 (通常 0.5 ~ 0.9)。
    // 2. ENERGY_LOSS_RATE (能量損失率): 遠點測試求出，透過平移虛擬目標點 B' 來彌補空氣阻力造成的動能流失 (通常為微小的正數)。
    // ==========================================
    public static double ENERGY_CONVERSION_RATE = 0.8472;
    public static double ENERGY_LOSS_RATE = 0.0002;
    public static double INTAKE_INITIAL_VELOCITY_IN_PER_SEC = 3;

    // ==========================================
    // 數學模型與反饋控制參數 (Math & Feedback Control)
    // ==========================================
    public static double IDEAL_INCIDENT_ANGLE_DEG = 30.0; // 目標 B 點的理想入射角 (度)，讓球能從上方完美掉入籃筐

    public static double RPM_ERROR_THRESHOLD = 100.0; // 反饋層的 RPM 誤差容忍度 (RPM)。當真實轉速與目標轉速相差過大時，不進行仰角微調以防伺服抖動
    public static double PITCH_OFFSET_DEG = 0.0; // 手動仰角微調偏移量 (度)
    // Physical offset of turret exit point from robot odometry center (inches)
    // Robot frame convention: +Y = forward, +X = left
    public static double TURRET_FORWARD_IN = 0.0;  // measure and fill in
    public static double TURRET_LEFT_IN    = 0.0;  // measure and fill in
}
