package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="WebcamServoSpinTest")
public class WebcamServoSpinTest extends OpMode {
    private Servo webcamServo;
    private double position = 0.5;  // Start in the middle (range 0–1)

    @Override
    public void init() {
        webcamServo = hardwareMap.get(Servo.class, "WebcamServo");
        telemetry.addLine("WebcamServoSpinTest ready");
        webcamServo.setPosition(0.5);
    }

    @Override
    public void loop() {
        // Use D-pad to adjust the servo position gradually
        if (gamepad2.dpad_right) position += 0.001;  // turn clockwise
        if (gamepad2.dpad_left)  position -= 0.001;  // turn counter-clockwise

        // Keep position between 0 and 1
        position = Math.max(0, Math.min(1, position));

        webcamServo.setPosition(position);

        telemetry.addData("Servo position", "%.2f", position);
        telemetry.update();
    }
}