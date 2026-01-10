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
    public double Kp = 0.014;        // <-- requested: single value for both Auto/TeleOp
    public double Kd = 0.002 ;
    public double deadband = 0.6;
    public double maxTurretPower = 1;

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
    public void setPipeline(String newPipeline) {
        if(newPipeline.equals("GPP"))
            pipeline = 0;
        else if(newPipeline.equals("PGP"))
            pipeline = 1;
        else if(newPipeline.equals("PPG"))
            pipeline = 2;
        else if(newPipeline.equals("Blue"))
            pipeline = 3;
        else if(newPipeline.equals("Red"))
            pipeline = 4;
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
    public void resetHistory(double runtimeSeconds) {
        lastTx = 0;
        lastAimTime = runtimeSeconds;
        lastTurretPower = 0;
    }

    /** Returns MOTOR power (already sign-corrected to match AprilTagLimelightTest). */
    public double update(double runtimeSeconds, Telemetry telemetry) {
        // ----- IMU -> Limelight orientation -----
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw() + yawOffsetDeg);

        // This variable matches AprilTagLimelightTest's "turretPower" meaning:
        // "PD output before the motor sign flip"
        double turretPower = 0;

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double error = -tx; // SAME as AprilTagLimelightTest

            double Aim_dt = runtimeSeconds - lastAimTime;
            if (Aim_dt <= 0) Aim_dt = 0.02;

            double dTx = (tx - lastTx) / Aim_dt;

            if (Math.abs(tx) <= deadband) {
                turretPower = 0;
            } else {
                turretPower = Kp * error - Kd * dTx; // SAME as AprilTagLimelightTest
            }

            turretPower = Range.clip(turretPower, -maxTurretPower, maxTurretPower);

            lastTx = tx;
            lastAimTime = runtimeSeconds;
            lastTurretPower = turretPower;

            if (telemetry != null) {
                telemetry.addData("AutoAim", "ON");
                telemetry.addData("pipeline", pipeline);
                telemetry.addData("Kp", Kp);
                telemetry.addData("Kd", Kd);
                telemetry.addData("tx", tx);
                telemetry.addData("ta", llResult.getTa());
                telemetry.addData("dTx", dTx);
                telemetry.addData("Turret PD", turretPower);     // same number you see in your test
                telemetry.addData("Motor Power", -turretPower);  // what actually gets applied
                telemetry.addData("IMU Yaw", orientation.getYaw());
            }
        } else {
            // SAME behavior as your test
            turretPower = lastTurretPower;

            if (telemetry != null) {
                telemetry.addData("AutoAim", "ON (no valid tag)");
                telemetry.addData("pipeline", pipeline);
                telemetry.addData("llResult", (llResult == null) ? "null" : "invalid");
                telemetry.addData("Motor Power", -turretPower);
            }
        }

        // IMPORTANT: return MOTOR power so Drivetrain2 can just setPower(returnValue)
        return -turretPower;
    }
}