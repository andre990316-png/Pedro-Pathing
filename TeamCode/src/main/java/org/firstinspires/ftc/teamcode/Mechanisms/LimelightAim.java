package org.firstinspires.ftc.teamcode.Mechanisms;

import static com.pedropathing.math.MathFunctions.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.controller.PIDController;
@Configurable
public class LimelightAim {
    private DcMotorEx ShooterRotateMotor;
    static double kp = 0.02, ki = 0.07, kd = 0.0012;
    public PIDController RotatePID = new PIDController(0, 0, 0);
    double RPM, rotatePower, lastRotatePower, manualPower = 0;
    double MaxRPM = 435;
    boolean manualEnabled = false;
    public static String currentPipeline;

    // ===== history =====
    public double lastTx = 0;
    public double lastAimTime = 0;
    public double lastTurretPower = 0;

    public void init(HardwareMap hardwareMap) {
        ShooterRotateMotor = hardwareMap.get(DcMotorEx.class, "ShooterRotateMotor");
        ShooterRotateMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ShooterRotateMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ShooterRotateMotor.setPower(0);
        RotatePID.setTolerance(0.041);
    }
    public void update(LLResult llResult, Telemetry telemetry, TelemetryManager telemetryM, double limelightAimPredictedGoalAngle) {
        if(manualEnabled) {
            ShooterRotateMotor.setPower(manualPower);
        }
        else {
            if (llResult != null && llResult.isValid()) {
                RPM = (ShooterRotateMotor.getVelocity() / 383.6) * 60.0;
                double Norm = RPM / MaxRPM;
                RotatePID.setPID(kp, ki, kd);
                rotatePower = RotatePID.calculate(llResult.getTx(), limelightAimPredictedGoalAngle);
                rotatePower = clamp(rotatePower, -1, 1);
                lastRotatePower = rotatePower;
                telemetry.addData("AutoAim", "ON");
            } else {
                rotatePower = lastRotatePower;
                telemetry.addData("AutoAim", "ON (no valid tag)");
                telemetry.addData("llResult", (llResult == null) ? "null" : "invalid");
            }
            ShooterRotateMotor.setPower(-rotatePower);
        }
        telemetry.addData("Current Pipeline", currentPipeline);
        telemetry.addData("Rotate Power", rotatePower);
        telemetry.addData("tx", llResult.getTx());
        telemetry.addData("ta", llResult.getTa());

        telemetryM.addData("Target Angle", 0);
        telemetryM.addData("Current Angle", llResult.getTx());
    }
    public void EnableManualPower(boolean enable, double power) {
        manualPower = power;
        manualEnabled = enable;
    }
    public static int pipelineFromName(String name) {
        currentPipeline = name;
        switch (name) {
            case "GPP": return 0;
            case "PGP": return 1;
            case "PPG": return 2;
            case "Blue": return 3;
            case "Red": return 4;
            case "Ramp": return 5;
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
}