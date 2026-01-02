package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoPositionSanity")
public class ServoPositionSanity extends OpMode {
    private Servo s;
    @Override public void init() {
        s = hardwareMap.get(Servo.class, "WebcamServo");
        s.setDirection(Servo.Direction.FORWARD); // try FORWARD first
        s.setPosition(0.50); // neutral
        telemetry.addLine("A=0.25  B=0.50  Y=0.75");
    }
    @Override public void loop() {
        if (gamepad2.a) s.setPosition(0.25);
        if (gamepad2.b) s.setPosition(0.50);
        if (gamepad2.y) s.setPosition(0.75);
        telemetry.addData("pos", "%.2f", 0.0); // just to force DS refresh
        telemetry.update();
    }
}