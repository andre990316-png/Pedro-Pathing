package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.bylazar.telemetry.PanelsTelemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;


@TeleOp(name = "PIDF RPM Control Web Panel")
@Configurable
public class PIDMotorRPMPanel extends OpMode {

    DcMotor leftMotor, rightMotor;
    PIDController pid;

    // --- PIDF values adjustable via web panel ---
    public static double kP = 0.0012;
    public static double kI = 0.000003;
    public static double kD = 0.0002;
    public static double kF = 0.000166;
    public static double frictionFactor = 0.05;

    public static double targetRPM = 0;

    int lastLeftPos = 0;
    int lastRightPos = 0;
    double lastTime = 0;

    // --- TelemetryManager for web panel ---
    TelemetryManager telemetryM;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "Shooter M1");
        rightMotor = hardwareMap.get(DcMotor.class, "Shooter M2");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(kP, kI, kD);

        // --- Initialize web panel telemetry ---
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {

        // --- Update PID values live from web panel ---
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);

        // --- Change target RPM with D-Pad ---
        if (gamepad1.dpad_up) {
            targetRPM += 500;
            sleepShort();
        }
        if (gamepad1.dpad_down) {
            targetRPM -= 500;
            sleepShort();
        }

        // --- Read motor RPMs ---
        double leftRPM = getRPM(leftMotor, true);
        double rightRPM = getRPM(rightMotor, false);
        double avgRPM = (leftRPM + rightRPM) / 2.0;

        // --- PID + feedforward + friction ---
        double power = pid.calculate(avgRPM, targetRPM);
        power += kF;
        power -= (avgRPM / 6000.0) * frictionFactor;
        power = Range.clip(power, -1, 1);

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        // --- Driver Station telemetry ---
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Left RPM", leftRPM);
        telemetry.addData("Right RPM", rightRPM);
        telemetry.addData("Avg RPM", avgRPM);
        telemetry.addData("Power", power);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.addData("Friction", frictionFactor);
        telemetry.update();

        // --- Web panel telemetry ---
        telemetryM.addData("TargetRPM", targetRPM);
        telemetryM.addData("LeftRPM", leftRPM);
        telemetryM.addData("RightRPM", rightRPM);
        telemetryM.addData("AvgRPM", avgRPM);
        telemetryM.addData("Power", power);
        telemetryM.addData("kP", kP);
        telemetryM.addData("kI", kI);
        telemetryM.addData("kD", kD);
        telemetryM.addData("kF", kF);
        telemetryM.addData("Friction", frictionFactor);
        telemetryM.update();
    }

    public double getRPM(DcMotor motor, boolean isLeft) {
        int pos = motor.getCurrentPosition();
        double time = getRuntime();
        int lastPos = isLeft ? lastLeftPos : lastRightPos;
        double deltaTime = time - lastTime;
        if (deltaTime == 0) deltaTime = 0.001; // avoid divide by zero
        double velocity = (pos - lastPos) / deltaTime;
        double rpm = velocity * 60 / 537.6; // GoBilda 312 RPM

        if (isLeft) lastLeftPos = pos;
        else lastRightPos = pos;
        lastTime = time;

        return rpm;
    }

    public void sleepShort() {
        try { Thread.sleep(200); } catch (Exception ignored) {}
    }
}