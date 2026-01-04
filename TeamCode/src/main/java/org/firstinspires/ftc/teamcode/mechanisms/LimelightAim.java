package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Shared Limelight + PD turret auto-aim.
 * Keeps your SAME variable names: Kp, Kd, deadband, maxTurretPower, lastTx, lastAimTime, lastTurretPower.
 * Uses "orientation" (not "o").
 */
public class LimelightAim {

    // Hardware
    public Limelight3A limelight;
    private final IMU imu;

    // ===== Your original variable names (values live here) =====
    public double Kp = 0.3;        // <-- requested: single value for both Auto/TeleOp
    public double Kd = 0.0025;
    public double deadband = 1.0;
    public double maxTurretPower = 1.0;

    public double lastTx = 0;
    public double lastAimTime = 0;
    public double lastTurretPower = 0;

    // Internal config
    private int pipeline = 3;       // <-- requested default
    private double yawOffsetDeg = 0;

    public LimelightAim(HardwareMap hardwareMap, IMU imu, String limelightName) {
        this.imu = imu;
        limelight = hardwareMap.get(Limelight3A.class, limelightName);

        limelight.pipelineSwitch(pipeline);

        lastTx = 0;
        lastAimTime = 0;
        lastTurretPower = 0;
    }

    /** Change pipeline from Auto/TeleOp whenever you want. */
    public void setPipeline(int pipeline) {
        this.pipeline = pipeline;
        limelight.pipelineSwitch(pipeline);
    }

    public int getPipeline() {
        return pipeline;
    }

    /** If your yaw needs a flip, do aim.setYawOffsetDeg(180). */
    public void setYawOffsetDeg(double yawOffsetDeg) {
        this.yawOffsetDeg = yawOffsetDeg;
    }

    public void start(double runtimeSeconds) {
        limelight.pipelineSwitch(pipeline); // re-assert
        limelight.start();
        lastAimTime = runtimeSeconds;
        lastTx = 0;
    }

    /** Returns turretPower (you still choose the sign when setting motor power). */
    public double update(double runtimeSeconds, Telemetry telemetry) {
        // ----- IMU -> Limelight orientation -----
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw() + yawOffsetDeg);

        double turretPower = 0;

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double error = -tx;

            double Aim_now = runtimeSeconds;
            double Aim_dt = Aim_now - lastAimTime;
            if (Aim_dt <= 0) Aim_dt = 0.02;

            double dTx = (tx - lastTx) / Aim_dt;

            if (Math.abs(tx) <= deadband) {
                turretPower = 0;
            } else {
                turretPower = Kp * error - Kd * dTx;
            }

            turretPower = Range.clip(turretPower, -maxTurretPower, maxTurretPower);

            lastTx = tx;
            lastAimTime = Aim_now;
            lastTurretPower = turretPower;

            if (telemetry != null) {
                telemetry.addData("AutoAim", "ON");
                telemetry.addData("pipeline", pipeline);
                telemetry.addData("Kp", Kp);
                telemetry.addData("Kd", Kd);
                telemetry.addData("tx", tx);
                telemetry.addData("ta", llResult.getTa());
                telemetry.addData("dTx", dTx);
                telemetry.addData("Turret PD", turretPower);
                telemetry.addData("IMU Yaw", orientation.getYaw());
            }
        } else {
            turretPower = lastTurretPower; // keep your current behavior

            if (telemetry != null) {
                telemetry.addData("AutoAim", "ON (no valid tag)");
                telemetry.addData("pipeline", pipeline);
                telemetry.addData("llResult", (llResult == null) ? "null" : "invalid");
            }
        }

        return turretPower;
    }
}