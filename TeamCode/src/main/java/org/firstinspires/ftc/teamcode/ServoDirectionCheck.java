package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoDirectionCheck")
public class ServoDirectionCheck extends OpMode {
    private Servo webcamServo;
    private double pos = 0.50;                // start centered

    @Override public void init() {
        webcamServo = hardwareMap.get(Servo.class, "WebcamServo");
        // Try FORWARD first; we’ll flip if needed.
        webcamServo.setDirection(Servo.Direction.FORWARD);
        webcamServo.setPosition(pos);
        telemetry.addLine("Press dpad RIGHT to increase pos, LEFT to decrease");
    }

    @Override public void loop() {
        if (gamepad2.dpad_right) pos += 0.01; // increase
        if (gamepad2.dpad_left)  pos -= 0.01; // decrease
        pos = Math.max(0.00, Math.min(1.00, pos));
        webcamServo.setPosition(pos);
        telemetry.addData("pos", "%.2f", pos);
        telemetry.update();
    }
}