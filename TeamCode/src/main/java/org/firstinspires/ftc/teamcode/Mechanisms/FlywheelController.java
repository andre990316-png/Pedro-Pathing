package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Configurable
public class FlywheelController {

    private DcMotorEx motor;

    // PDF control gains (Proportional, Derivative, Feed-forward)
    private double kp = 0.015;
    private double kd = 0.0001;
    private double kf = 0.05;

    private int ticksPerRevolution = 28;
    private double maxRPM = 6000;

    private double targetRPM = 0;
    private double currentRPM = 0;
    private double previousError = 0;

    // Data logging
    private List<Double> rpmHistory = new ArrayList<>();
    private List<Double> targetHistory = new ArrayList<>();
    private List<Double> timeHistory = new ArrayList<>();
    private ElapsedTime timer = new ElapsedTime();
    private double lastLogTime = 0;

    public FlywheelController() {}

    public void init(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        timer.reset();
    }

    public void update() {
        // Get current RPM
        double ticksPerSecond = motor.getVelocity();
        currentRPM = (ticksPerSecond / ticksPerRevolution) * 60.0;

        // PDF control (Proportional, Derivative, Feed-forward)
        double error = targetRPM - currentRPM;
        double derivative = (error - previousError) / 0.02;

        double pTerm = kp * error;
        double dTerm = kd * derivative;
        double fTerm = kf * (targetRPM / maxRPM);

        double power = pTerm + dTerm + fTerm;
        power = Range.clip(power, -1.0, 1.0);

        motor.setPower(power);
        previousError = error;

        // Log data
        logData();
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0, maxRPM);
        previousError = 0;
    }

    public void stop() {
        targetRPM = 0;
    }

    private void logData() {
        double currentTime = timer.seconds();
        if (currentTime - lastLogTime >= 0.05) {
            rpmHistory.add(currentRPM);
            targetHistory.add(targetRPM);
            timeHistory.add(currentTime);
            lastLogTime = currentTime;

            if (rpmHistory.size() > 300) {
                rpmHistory.remove(0);
                targetHistory.remove(0);
                timeHistory.remove(0);
            }
        }
    }

    public void sendTelemetry(TelemetryManager telemetryM) {
        if (telemetryM == null) return;

        telemetryM.addData("flywheel_current_rpm", currentRPM);
        telemetryM.addData("flywheel_target_rpm", targetRPM);
        telemetryM.addData("flywheel_error", targetRPM - currentRPM);
        telemetryM.addData("flywheel_power", motor.getPower());
    }

    public double getCurrentRPM() { return currentRPM; }
    public double getTargetRPM() { return targetRPM; }
    public boolean isAtTarget() { return Math.abs(targetRPM - currentRPM) <= 100; }

    // Getters/Setters for Panels tuning
    public double getKp() { return kp; }
    public void setKp(double kp) { this.kp = kp; }
    public double getKd() { return kd; }
    public void setKd(double kd) { this.kd = kd; }
    public double getKf() { return kf; }
    public void setKf(double kf) { this.kf = kf; }
}