package org.firstinspires.ftc.teamcode.Tests;

import static android.os.SystemClock.sleep;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class PIDTest extends OpMode {
    protected DcMotorEx shooterL, shooterR, intakeL, intakeR;
    Servo servo;
    Servo HoodServo;
    TelemetryManager telemetryM;
    double lVelocity, shooter_power;
    public PIDController ShooterLPID = new PIDController(0, 0, 0);
    static double sP = 4.7, sI = 0.07, sD = 0.4;
    static double targetVelocity;
    double MaxFlywheelVelocity = 6000;
    double iRPM, intake_power;
    static double iP = 0.0065, iI = 0, iD = 0.00004;
    public PIDController IntakePID = new PIDController(0, 0, 0);
    static double targetRPM = 1620;
    double MaxIntakeRPM = 1620;
    static double triggerOpenPos = 0.295, triggerClosedPos = 0;
    static double hoodPos = 0.95;
    static boolean justPressed = false;

    @Override
    public void init() {
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");
        intakeL = hardwareMap.get(DcMotorEx.class, "Intake1");
        intakeR = hardwareMap.get(DcMotorEx.class, "Intake2");
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setPower(0);
        shooterR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setPower(0);
        intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setPower(0);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setPower(0);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo = hardwareMap.get(Servo.class, "Trig");
        HoodServo = hardwareMap.get(Servo.class, "LV");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        double velocity1 = targetVelocity / MaxFlywheelVelocity;
        lVelocity = (shooterR.getVelocity() / 28.0) * 60.0;
        double norm = lVelocity / MaxFlywheelVelocity;
        ShooterLPID.setPID(sP, sI, sD); // setting PID
        shooter_power = ShooterLPID.calculate(norm, velocity1); // calculate output power
        shooter_power = clamp(shooter_power + velocity1, -1.0, 1.0); // setting in correct range
        shooterR.setPower(shooter_power);
        shooterL.setPower(shooter_power);

        double velocity = targetRPM / MaxIntakeRPM;
        iRPM = (intakeR.getVelocity() / 103.6) * 60.0;
        double Inorm = iRPM / MaxIntakeRPM;
        IntakePID.setPID(iP, iI, iD);
        intake_power = IntakePID.calculate(Inorm, velocity);
        intake_power = MathFunctions.clamp(intake_power + velocity, -1.0, 1.0);

        HoodServo.setPosition(clamp(hoodPos, 0.3, 0.95));
        if(!gamepad1.circle && justPressed) {
            justPressed = false;
        }
        if (gamepad1.circle && !justPressed) {
            servo.setPosition(triggerOpenPos);
            sleep(500);
            justPressed = true;
        }
        if (!justPressed) {
            servo.setPosition(triggerClosedPos);
        }
        if (gamepad1.left_bumper || gamepad1.circle) {
            intakeR.setPower(1);
            intakeL.setPower(1);
        } else {
            intakeR.setPower(0);
            intakeL.setPower(0);
            IntakePID.reset();
        }
        telemetryM.addData("shooterVelocity", (shooterR.getVelocity() / 28) * 60);
        telemetryM.addData("shooterTargetVelocity", targetVelocity);
        telemetryM.addData("intakeVelocity", (intakeR.getVelocity() / 103.6) * 60);
        telemetryM.addData("intakeTargetVelocity", targetRPM);
        telemetryM.addData("servo pos", servo.getPosition());

        telemetryM.update();
    }
}