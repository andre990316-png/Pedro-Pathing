package org.firstinspires.ftc.teamcode.Tests;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mechanisms.SortLogic;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Mechanisms.DualBallColorSensor;

import org.firstinspires.ftc.teamcode.Mechanisms.ButtonLogic;
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
    private DualBallColorSensor upBallSensor;
    private DualBallColorSensor downBallSensor;
    static double gateOpenAngle = 0.22, gateCloseAngle = 0, sortOpenAngle = 0.2, sortCloseAngle = 1;
    public static int[] colors = {0, 0}; //0=none, 1=purple, 2=green

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
    private ButtonLogic autoSortBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);

    private SortLogic sortLogic = new SortLogic();
    private ElapsedTime sortTimer = new ElapsedTime();
    static int pattern = 1; // 1 = target PPG (2 purple + 1 green)

    @Override
    public void init() {
        ShooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        ShooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        IntakeR  = hardwareMap.get(DcMotorEx.class, "intakeR");
        IntakeL  = hardwareMap.get(DcMotorEx.class, "intakeL");
        ShooterServo = hardwareMap.get(Servo.class, "servo");
        SortServo1 = hardwareMap.get(Servo.class, "sortServo1");
        SortServo2 = hardwareMap.get(Servo.class, "sortServo2");
        upBallSensor = new DualBallColorSensor(hardwareMap, "colorSensorUp1", "colorSensorUp2", 16.0f);
        downBallSensor = new DualBallColorSensor(hardwareMap, "colorSensorDown1", "colorSensorDown2", 16.0f);

        IntakeR.setDirection(DcMotor.Direction.REVERSE);

        ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterServo.setPosition(gateCloseAngle);
        SortServo1.setPosition(0);
        SortServo2.setPosition(0);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        lastShooterPos = (ShooterR.getCurrentPosition() + ShooterL.getCurrentPosition()) / 2;
        lastTimeNs = System.nanoTime();
        sortTimer.reset();
        sortLogic.reset(sortTimer.seconds());
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
        autoSortBtn.update(gamepad1.left_trigger >= 0.3);

        if (intakeBtn.getState()) {
            IntakeL.setPower(-1);
            IntakeR.setPower(-1);
        } else {
            IntakeL.setPower(0);
            IntakeR.setPower(0);
        }
        if(rpmUpBtn.getState())
            RPM += 50;
        if(rpmDownBtn.getState())
            RPM -= 50;
        if(servoTestBtnUp.getState())
            sortOpenAngle += 0.1;
        if(servoTestBtnDown.getState())
            sortOpenAngle -= 0.1;
//        if(sortServoBtnUp.getState())
//            SortServo1.setPosition(sortOpenAngle);
//        else
//            SortServo1.setPosition(sortCloseAngle);
//        if(sortServoBtnDown.getState())
//            SortServo2.setPosition(sortOpenAngle);
//        else
//            SortServo2.setPosition(sortCloseAngle);

        if (!autoSortBtn.getState()) {
            if(sortServoBtnUp.getState()) SortServo1.setPosition(sortOpenAngle);
            else SortServo1.setPosition(sortCloseAngle);

            if(sortServoBtnDown.getState()) SortServo2.setPosition(sortOpenAngle);
            else SortServo2.setPosition(sortCloseAngle);
        }

        if (shooterBtn.getState()) {
            targetRPM = RPM;
        } else {
            targetRPM = 0;
        }

        double shooterPower = updateShooterPD(targetRPM);

        ShooterR.setPower(-shooterPower);
        ShooterL.setPower(shooterPower);

        if (shootBtn.getState()) {
            ShooterServo.setPosition(gateOpenAngle);
        } else {
            ShooterServo.setPosition(gateCloseAngle);
        }

        telemetry.addData("Target RPM", RPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Error", (targetRPM - currentRPM));
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Gate Servo Open Angle", gateOpenAngle);
        telemetry.addData("Sort Servo Open Angle", sortOpenAngle);
        telemetry.addData("Sort Up/Down", "%b %b", sortLogic.isOpenUpSort(), sortLogic.isOpenDownSort());
        telemetry.addData("Sort feed/shoot", "%b %b", sortLogic.isFeedOn(), sortLogic.isShootOn());
        telemetry.addData("Colors", "%d %d", colors[0], colors[1]);

        //Color Logic

        upBallSensor.update();
        colors[0] = upBallSensor.getBallColor();

        downBallSensor.update();
        colors[1] = downBallSensor.getBallColor();

        sortLogic.update(colors, pattern);

// triggerSort = you decide how to start sorting
// Example: press X to perform sorting until done (you can also make it auto)
        boolean triggerSort = gamepad1.x;

        sortLogic.step(sortTimer.seconds(), triggerSort);

// Apply pocket commands to your two sort servos
        SortServo1.setPosition(sortLogic.isOpenUpSort() ? sortOpenAngle : sortCloseAngle);
        SortServo2.setPosition(sortLogic.isOpenDownSort() ? sortOpenAngle : sortCloseAngle);

// Apply feed command to intake motors
        if (sortLogic.isFeedOn()) {
            IntakeL.setPower(-1);
            IntakeR.setPower(-1);
        }

// Apply shoot command to gate servo
        if (sortLogic.isShootOn()) {
            ShooterServo.setPosition(gateOpenAngle);
        } else if (!shootBtn.getState()) {
            ShooterServo.setPosition(gateCloseAngle);
        }

        upBallSensor.telemetry(telemetry, "UP");
        downBallSensor.telemetry(telemetry, "DOWN");

//        upBallSensor.telemetry(telemetryM, "UP");
//        downBallSensor.telemetry(telemetryM, "DOWN");

        telemetry.update();
        telemetryM.update();
    }

//    @Override
//    public void stop() {
//
//    }

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