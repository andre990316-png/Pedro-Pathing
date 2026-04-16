package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motor_power")
public class motor_power extends OpMode {

    DcMotor motor1;
    DcMotor motor2;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "Shooter M1");
        motor2 = hardwareMap.get(DcMotor.class, "Shooter M2");

        // Reverse one motor if needed (depends on your build)
        motor2.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        // Get joystick value (invert because up is -1)
        double power = -gamepad1.left_stick_y;

        // Dead zone to prevent drifting
        if (Math.abs(power) < 0.05) {
            power = 0;
        }

        // Set BOTH motors to same power
        motor1.setPower(power);
        motor2.setPower(power);

        telemetry.addData("Power", power);
        telemetry.update();
    }
}