package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.arcrobotics.ftclib.controller.PIDController;


@TeleOp(name = "PIDF RPM Control")
public class PIDMotorRPM extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    // --- PID values (tune these) ---


    PIDController pid = new PIDController(0.0012, 0.000003, 0.0002);

    // --- Feedforward to overcome friction and voltage loss ---
    double kF = 0.000166;   // adjust for your motor / flywheel
    double frictionFactor = 0.05; // simulated friction

    double targetRPM = 0;

    int lastLeftPos = 0;
    int lastRightPos = 0;
    double lastTime = 0;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "Shooter M1");
        rightMotor = hardwareMap.get(DcMotor.class, "Shooter M2");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        // --- Change target RPM with D-Pad ---
        if (gamepad1.dpad_up) {
            targetRPM += 500;
            sleepShort();
        }
        if (gamepad1.dpad_down) {
            targetRPM -= 500;
            sleepShort();
        }

        // --- Read current RPM ---
        double leftRPM = getRPM(leftMotor, true);
        double rightRPM = getRPM(rightMotor, false);

        double currentRPM = (leftRPM + rightRPM) / 2.0; // average for PID

        // --- PID output ---
        double power = pid.calculate(currentRPM, targetRPM);

        // --- Add feedforward and friction ---
        power += kF;  // feedforward to overcome static friction
        power -= (currentRPM / 6000.0) * frictionFactor; // simulated friction

        // --- Clamp power to [-1,1] ---
        power = Math.max(-1, Math.min(1, power));

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        // --- Telemetry ---
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Left RPM", leftRPM);
        telemetry.addData("Right RPM", rightRPM);
        telemetry.addData("Average RPM", currentRPM);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    // --- Get RPM from a motor ---
    public double getRPM(DcMotor motor, boolean isLeft) {
        int position = motor.getCurrentPosition();
        double time = getRuntime();

        int lastPos = isLeft ? lastLeftPos : lastRightPos;

        double velocity = (position - lastPos) / (time - lastTime); // ticks/sec
        double rpm = velocity * 60 / 537.6; // GoBilda 312RPM motor

        if (isLeft) lastLeftPos = position;
        else lastRightPos = position;

        lastTime = time;

        return rpm;
    }

    // --- Small delay to avoid repeated D-Pad inputs ---
    public void sleepShort() {
        try {
            Thread.sleep(200);
        } catch (Exception e) {}
    }
}