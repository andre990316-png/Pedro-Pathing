package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@Configurable
@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode {

    private Servo servo;

    // adjustable
    static double pos = 0.5;
    private double stepFine = 0.01;      // dpad step
    private double speed = 0.5;          // stick speed (pos/sec)

    private boolean lastUp = false, lastDown = false, lastA = false, lastB = false, lastX = false;
    private static String servoName = "PTO";

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, servoName);
        servo.setPosition(pos);

        telemetry.addLine("ServoTest init OK");
        telemetry.addLine("Left stick Y: move | Dpad up/down: fine | X=min A=center B=max");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---- Smooth move with stick ----
        double stick = -gamepad1.left_stick_y; // up = positive
        double dt = Math.max(0.0, (double) getRuntime() - lastRuntime);
        lastRuntime = getRuntime();

        pos += stick * speed * dt;

        // ---- Fine adjust with dpad (edge-triggered) ----
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (up && !lastUp) pos += stepFine;
        if (down && !lastDown) pos -= stepFine;

        lastUp = up;
        lastDown = down;

        // ---- Presets (edge-triggered) ----
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;

        if (a && !lastA) pos = 0.5;
        if (x && !lastX) pos = 0.0;
        if (b && !lastB) pos = 1.0;

        lastA = a;
        lastB = b;
        lastX = x;

        // Clamp + apply
        pos = Range.clip(pos, 0.0, 1.0);
        servo.setPosition(pos);

        // Telemetry
        telemetry.addData("Servo Name", servoName);
        telemetry.addData("Commanded Pos", "%.4f", pos);
        telemetry.addData("Servo.getPosition()", "%.4f", servo.getPosition());
        telemetry.addData("Stick", "%.3f", stick);
        telemetry.update();
    }

    private double lastRuntime = 0.0;
}