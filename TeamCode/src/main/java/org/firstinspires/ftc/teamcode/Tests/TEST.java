package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mechanisms.ButtonLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.DualColorSensor;
import org.firstinspires.ftc.teamcode.Mechanisms.SingleColorSensor;
import org.firstinspires.ftc.teamcode.Mechanisms.SortLogic;

@Configurable
@TeleOp
public class TEST extends OpMode {

    public TelemetryManager telemetryM;

    private DcMotorEx ShooterR;
    private DcMotorEx ShooterL;
    private DcMotorEx IntakeR;
    private DcMotorEx IntakeL;

    private Servo ShooterServo;
    private Servo SortServo1;
    private Servo SortServo2;

    private SingleColorSensor upBallSensor;
    private SingleColorSensor downBallSensor;

    static double gateOpenAngle = 0.22, gateCloseAngle = 0, sortOpenAngle = 0, sortCloseAngle = 1, resetAngle = 0;

    public static int[] colors = {0, 0}; // 0=none, 1=purple, 2=green

    static double Kp = 0.0025;
    static double Kd = 0.0002;

    private static final double TICKS_PER_REV = 28.0;
    private int lastShooterPos = 0;
    private long lastTimeNs = 0;

    private double targetRPM = 0.0;
    static double RPM = 0.0;
    private double currentRPM = 0.0;
    private double lastError = 0.0;

    private ButtonLogic intakeBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic shooterBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic shootBtn  = new ButtonLogic(ButtonLogic.Mode.HOLD, false);
    private ButtonLogic rpmUpBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic rpmDownBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic servoTestBtnUp = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic servoTestBtnDown = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic sortServoBtnUp = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic sortServoBtnDown = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic sortServoResetBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);

    private final SortLogic sortLogic = new SortLogic();
    private final ElapsedTime Timer = new ElapsedTime();
    static int pattern = 1; // target PPG for 2P+1G

    @Override
    public void init() {
        ShooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        ShooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        IntakeR  = hardwareMap.get(DcMotorEx.class, "intakeR");
        IntakeL  = hardwareMap.get(DcMotorEx.class, "intakeL");

        ShooterServo = hardwareMap.get(Servo.class, "servo");
        SortServo1 = hardwareMap.get(Servo.class, "sortServo1");
        SortServo2 = hardwareMap.get(Servo.class, "sortServo2");

        upBallSensor = new SingleColorSensor(hardwareMap, "colorSensorUp1", 16.0f);
        downBallSensor = new SingleColorSensor(hardwareMap, "colorSensorUp2", 16.0f);

        IntakeR.setDirection(DcMotor.Direction.REVERSE);

        ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterServo.setPosition(gateCloseAngle);
        SortServo1.setPosition(sortCloseAngle);
        SortServo2.setPosition(sortCloseAngle);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        lastShooterPos = (ShooterR.getCurrentPosition() + ShooterL.getCurrentPosition()) / 2;
        lastTimeNs = System.nanoTime();

        Timer.reset();
        sortLogic.reset(Timer.seconds());
    }

    @Override
    public void loop() {
        intakeBtn.update(gamepad1.left_bumper);
        shootBtn.update(gamepad1.right_trigger >= 0.3);
        shooterBtn.update(gamepad1.right_bumper);
        rpmUpBtn.update(gamepad1.dpad_up);
        rpmDownBtn.update(gamepad1.dpad_down);
        servoTestBtnUp.update(gamepad1.dpad_right);
        servoTestBtnDown.update(gamepad1.dpad_left);
        sortServoBtnUp.update(gamepad1.a);
        sortServoBtnDown.update(gamepad1.b);
        sortServoResetBtn.update(gamepad1.x);

        if (rpmUpBtn.getState()) RPM += 50;
        if (rpmDownBtn.getState()) RPM -= 50;

        if (servoTestBtnUp.getState()) sortOpenAngle += 0.1;
        if (servoTestBtnDown.getState()) sortOpenAngle -= 0.1;
        sortOpenAngle = Range.clip(sortOpenAngle, 0.0, 1.0);

        if (shooterBtn.getState()) targetRPM = RPM;
        else targetRPM = 0;

        double shooterPower = updateShooterPD(targetRPM);
        ShooterR.setPower(-shooterPower);
        ShooterL.setPower(shooterPower);

        upBallSensor.update();
        colors[0] = upBallSensor.getBallColor();
        downBallSensor.update();
        colors[1] = downBallSensor.getBallColor();

        sortLogic.update(colors, pattern);
        sortLogic.step(Timer.seconds(), shootBtn.getState());
        boolean sortingActive = sortLogic.isBusy();

        if (sortingActive) {
            SortServo1.setPosition(sortLogic.isOpenUpSort() ? sortOpenAngle : sortCloseAngle);
            SortServo2.setPosition(sortLogic.isOpenDownSort() ? sortOpenAngle : sortCloseAngle);

            double feedPower = sortLogic.isFeedOn() ? -1.0 : 0.0;
            IntakeL.setPower(feedPower);
            IntakeR.setPower(feedPower);

            ShooterServo.setPosition(sortLogic.isShootOn() ? gateOpenAngle : gateCloseAngle);
        } else {
            if(sortServoResetBtn.getState()) {
                SortServo1.setPosition(resetAngle);
                SortServo2.setPosition(resetAngle);
            }
            else {
                SortServo1.setPosition(sortServoBtnUp.getState() ? sortOpenAngle : sortCloseAngle);
                SortServo2.setPosition(sortServoBtnDown.getState() ? sortOpenAngle : sortCloseAngle);
            }

            double intakePower = intakeBtn.getState() ? -1.0 : 0.0;
            IntakeL.setPower(intakePower);
            IntakeR.setPower(intakePower);

            ShooterServo.setPosition(shootBtn.getState() ? gateOpenAngle : gateCloseAngle);
        }

        telemetry.addData("Target RPM", RPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("ShooterR Position", ShooterR.getCurrentPosition());
        telemetry.addData("ShooterL Position", ShooterL.getCurrentPosition());

        telemetry.addData("Colors", "%d %d", colors[0], colors[1]);
        telemetry.addData("Sorting", sortingActive);
        telemetry.addData("Sort Up/Down", "%b %b", sortLogic.isOpenUpSort(), sortLogic.isOpenDownSort());
        telemetry.addData("Sort feed/shoot", "%b %b", sortLogic.isFeedOn(), sortLogic.isShootOn());

        upBallSensor.telemetry(telemetry, "UP");
        downBallSensor.telemetry(telemetry, "DOWN");

        telemetry.update();
        telemetryM.update();
    }

    private double updateShooterPD(double targetRPM) {
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastTimeNs) / 1e9;
        if (dt <= 0) dt = 0.02;
        if (dt > 0.2) dt = 0.02;

        int pos = (ShooterR.getCurrentPosition() + ShooterL.getCurrentPosition()) / 2;
        int dPos = pos - lastShooterPos;
        dPos = -dPos;

        double rev = dPos / TICKS_PER_REV;
        currentRPM = (rev / dt) * 60.0;

        lastShooterPos = pos;
        lastTimeNs = nowNs;

        double error = targetRPM - currentRPM;
        double dError = (error - lastError) / dt;
        lastError = error;

        double power = (Kp * error) + (Kd * dError);
        if (targetRPM <= 1) power = 0;

        return Range.clip(power, 0.0, 1.0);
    }
}