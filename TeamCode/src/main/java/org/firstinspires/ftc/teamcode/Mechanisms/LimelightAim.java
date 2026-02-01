package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightAim {

    // ===== tunables =====
    public double Kp = 0.017 ;
    public double Kd = 0.0003;
    public double deadband = 0.1;
    public double maxTurretPower = 1.0;
    public double minTurretPower = 0.09;
    public static String currentPipeline;

    // ===== history =====
    public double lastTx = 0;
    public double lastAimTime = 0;
    public double lastTurretPower = 0;

    /** Convert your pipeline name -> number (so opmodes can call pipelineSwitch(int)). */
    public static int pipelineFromName(String name) {
        currentPipeline = name;
        switch (name) {
            case "GPP": return 0;
            case "PGP": return 1;
            case "PPG": return 2;
            case "Blue": return 3;
            case "Red": return 4;
            default: return 3;
        }
    }
    public static String getCurrentPipeline(String name) {
        return currentPipeline;
    }
    public static int getAprilTagId(LLResult llResult) {
        int id = 0;
        if (llResult != null && llResult.isValid()) {
            java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty())
                id = fiducials.get(0).getFiducialId();
        }
        return id;
    }

    public void resetHistory(double runtimeSeconds) {
        lastTx = 0;
        lastAimTime = runtimeSeconds;
        lastTurretPower = 0;
    }

    /**
     * Returns MOTOR power (already sign-corrected like your AprilTagLimelightTest):
     * your OpMode can directly do ShooterRotateMotor.setPower(returnValue).
     */
    public double update(double runtimeSeconds, LLResult llResult, Telemetry telemetry) {

        double turretPower = 0;

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double error = -tx; // same as your working test

            double Aim_dt = runtimeSeconds - lastAimTime;
            if (Aim_dt <= 0.02) Aim_dt = 0.02;

            double dTx = (tx - lastTx) / Aim_dt;

            if (Math.abs(tx) <= deadband) {
                turretPower = 0;
            } else {
                turretPower = Kp * error - Kd * dTx;
            }

            turretPower = Range.clip(turretPower, -maxTurretPower, maxTurretPower);

            if (Math.abs(tx) > deadband && Math.abs(turretPower) < minTurretPower) {
                turretPower = Math.copySign(minTurretPower, turretPower);
            }

            lastTx = tx;
            lastAimTime = runtimeSeconds;
            lastTurretPower = turretPower;

            if (telemetry != null) {
                telemetry.addData("AutoAim", "ON");
                telemetry.addData("tx", llResult.getTx());
                telemetry.addData("ta", llResult.getTa());
                telemetry.addData("dTx", dTx);
                telemetry.addData("Turret PD", turretPower);
                telemetry.addData("Motor Power", -turretPower);
            }

        } else {
            turretPower = lastTurretPower;

            if (telemetry != null) {
                telemetry.addData("AutoAim", "ON (no valid tag)");
                telemetry.addData("llResult", (llResult == null) ? "null" : "invalid");
                telemetry.addData("Motor Power", -turretPower);
            }
        }
        telemetry.addData("Current Pipeline", currentPipeline);
        // motor sign matches your test: ShooterRotateMotor.setPower(-turretPower)
        return -turretPower;
    }
}