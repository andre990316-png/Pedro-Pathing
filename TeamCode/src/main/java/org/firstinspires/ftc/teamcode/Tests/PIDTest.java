package org.firstinspires.ftc.teamcode.Tests;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class PIDTest extends OpMode {
    protected DcMotorEx shooterL, shooterR, intakeL, intakeR;
    Servo servo;
    TelemetryManager telemetryM;
    double rVelocity, lVelocity, shooterR_power, shooterL_power;
    public PIDController ShooterRPID = new PIDController(0, 0, 0);
    public PIDController ShooterLPID = new PIDController(0, 0, 0);
    static double rP = 5, rI = 0.27, rD = 0, lP = 5, lI = 0.27, lD = 0;
    static double targetVelocity, intakePower = 1, position;
    double MaxLeftVelocity = 6000, MaxRightVelocity = 6000;

    @Override
    public void init() {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        intakeL = hardwareMap.get(DcMotorEx.class, "intakeL");
        intakeR = hardwareMap.get(DcMotorEx.class, "intakeR");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setPower(0);
        shooterR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setPower(0);
        servo = hardwareMap.get(Servo.class, "servo");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        double velocity1 = targetVelocity / MaxRightVelocity;
        double velocity2 = targetVelocity / MaxLeftVelocity;
        rVelocity = ((shooterR.getVelocity() / 28) * 60) / MaxRightVelocity;
        lVelocity = ((shooterL.getVelocity() / 28) * 60) / MaxLeftVelocity;
        ShooterRPID.setPID(rP, rI, rD); // setting PID
        shooterR_power = ShooterRPID.calculate(rVelocity, velocity1); // calculate output power
        ShooterLPID.setPID(lP, lI, lD); // setting PID
        shooterL_power = ShooterLPID.calculate(lVelocity, velocity2); // calculate output power
        shooterR_power = clamp(shooterR_power + velocity1, -1, 1.0); // setting in correct range
        shooterL_power = clamp(shooterL_power + velocity2, -1, 1.0);
        shooterR.setPower(shooterR_power);
        shooterL.setPower(shooterL_power);
        if (gamepad1.left_bumper || gamepad1.circle) {
            intakeR.setPower(intakePower);
            intakeL.setPower(intakePower);
        }
        if (gamepad1.circle) {
            servo.setPosition(0.22);
        } else {
            servo.setPosition(0);
        }
        telemetryM.addData("Rvelocity", (shooterR.getVelocity() / 28) * 60);
        telemetryM.addData("Lvelocity", (shooterL.getVelocity() / 28) * 60);
        telemetryM.addData("targetVelocitty", targetVelocity);
        telemetryM.update();
    }
}