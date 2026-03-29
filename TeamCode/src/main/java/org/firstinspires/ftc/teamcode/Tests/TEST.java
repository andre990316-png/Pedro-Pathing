package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Data.AllianceData;
import org.firstinspires.ftc.teamcode.Mechanisms.ButtonLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.DualColorSensor;
import org.firstinspires.ftc.teamcode.Mechanisms.SingleColorSensor;
import org.firstinspires.ftc.teamcode.Mechanisms.NewSortLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.RampBallSequencer;

import java.util.ArrayList;
import java.util.List;

@Configurable
@TeleOp
public class TEST extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    public TelemetryManager telemetryM;
    private PIDController shooterPID;

    private DcMotorEx ShooterR;
    private DcMotorEx ShooterL;
    private DcMotorEx IntakeR;
    private DcMotorEx IntakeL;

    private Servo ShooterServo;
    private Servo SortServo1;
    private Servo SortServo2;

    private SingleColorSensor firstBallSensor;
    private SingleColorSensor secondBallSensor;
    private SingleColorSensor thirdBallSensor;
    
//    private DualColorSensor firstBallSensor;
//    private DualColorSensor secondBallSensor;
//    private DualColorSensor thirdBallSensor;

    static double gateOpenAngle = 0.2, gateCloseAngle = 0, sortOpenAngle = 0, sortCloseAngle = 1, resetAngle = 0;

    public static int[] colors = {0, 0, 0}; // 0=none, 1=purple, 2=green

    static double Kp = 0.0018;
    static double Kd = 0.0005;
    static double Ki = 0;

    private static final double TICKS_PER_REV = 28.0;
    private int lastShooterPos = 0;
    private long lastTimeNs = 0;
    private double maxShooterRPM = 5400;

    private double targetRPM = 0.0;
    static double RPM = 0.0;
    private double currentRPM = 0.0;
    private double lastError = 0.0;

    private ButtonLogic intakeBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic reverseIntakeBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic shooterBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic shootBtn  = new ButtonLogic(ButtonLogic.Mode.HOLD, false);
    private ButtonLogic rpmUpBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic rpmDownBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic servoTestBtnUp = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic servoTestBtnDown = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic sortServoBtnUp = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic sortServoBtnDown = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic sortServoResetBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic patternBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic rampDetectionBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private final NewSortLogic sortLogic = new NewSortLogic();
    private final ElapsedTime Timer = new ElapsedTime();
    static int pattern = 1; // target PPG for 2P+1G
    private RampBallSequencer rampBallSequencer = new RampBallSequencer();
    private Pose blueScanRampPose = new Pose(50, 70);
    private Pose redScanRampPose = new Pose(94, 70);
    private List<RampBallSequencer.BallObs> rampBallColors = new ArrayList<>();

    @Override
    public void init() {
        ShooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        ShooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        IntakeR  = hardwareMap.get(DcMotorEx.class, "intakeR");
        IntakeL  = hardwareMap.get(DcMotorEx.class, "intakeL");

        ShooterServo = hardwareMap.get(Servo.class, "servo");
        SortServo1 = hardwareMap.get(Servo.class, "sortServo1");
        SortServo2 = hardwareMap.get(Servo.class, "sortServo2");

        //firstBallSensor = new DualColorSensor(hardwareMap, "colorSensorFirst1", "colorSensorFirst2", 16.0f);
        //secondBallSensor = new DualColorSensor(hardwareMap, "colorSensorSecond1", "colorSensorSecond2", 16.0f);
        //thirdBallSensor = new DualColorSensor(hardwareMap, "colorSensorThird1", "colorSensorThird2", 16.0f);

        firstBallSensor = new SingleColorSensor(hardwareMap, "colorSensorFirst1", 16.0f);
        secondBallSensor = new SingleColorSensor(hardwareMap, "colorSensorSecond1", 16.0f);
        thirdBallSensor = new SingleColorSensor(hardwareMap, "colorSensorThird1", 16.0f);

        IntakeR.setDirection(DcMotor.Direction.REVERSE);

        ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterServo.setPosition(gateCloseAngle);
        SortServo1.setPosition(sortCloseAngle);
        SortServo2.setPosition(sortCloseAngle);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        lastShooterPos = (Math.abs(ShooterR.getCurrentPosition()) + Math.abs(ShooterL.getCurrentPosition())) / 2;
        lastTimeNs = System.nanoTime();

        shooterPID = new PIDController(Kp, Ki, Kd);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        Timer.reset();
        sortLogic.reset(Timer.seconds());
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        intakeBtn.update(gamepad1.left_bumper);
        reverseIntakeBtn.update(gamepad1.left_trigger > 0.3);
        shooterBtn.update(gamepad1.right_bumper);
        shootBtn.update(gamepad1.right_trigger >= 0.3);
        rpmUpBtn.update(gamepad1.dpad_up);
        rpmDownBtn.update(gamepad1.dpad_down);
        servoTestBtnUp.update(gamepad1.dpad_right);
        servoTestBtnDown.update(gamepad1.dpad_left);
        sortServoBtnUp.update(gamepad1.a);
        sortServoBtnDown.update(gamepad1.b);
        sortServoResetBtn.update(gamepad1.x);
        patternBtn.update(gamepad1.y);

        LLResult ll = limelight.getLatestResult();
        if (rampDetectionBtn.getState()) {
            rampBallSequencer.update(blueScanRampPose, ll, AllianceData.isRed(), telemetry, telemetryM);
            rampBallColors = rampBallSequencer.getOnRampBalls();
            // Example: print the ordered ramp sequence colors
            for (int i = 0; i < rampBallSequencer.getOnRampBalls().size(); i++) {
                telemetry.addData("RampSeq " + i, rampBallSequencer.getOnRampBalls().get(i).color);
            }
        }

        if(patternBtn.getState()) {
            pattern += 1;
            pattern %= 4;
        }

        if (rpmUpBtn.getState()) RPM += 50;
        if (rpmDownBtn.getState()) RPM -= 50;

        if (servoTestBtnUp.getState()) sortOpenAngle += 0.1;
        if (servoTestBtnDown.getState()) sortOpenAngle -= 0.1;
        sortOpenAngle = Range.clip(sortOpenAngle, 0.0, 1.0);

        if (shooterBtn.getState()) targetRPM = RPM;
        else targetRPM = 0;

        double shooterPower = updateShooterPID(targetRPM);
        ShooterR.setPower(-shooterPower);
        ShooterL.setPower(shooterPower);

        firstBallSensor.update();
        colors[0] = firstBallSensor.getBallColor();
        secondBallSensor.update();
        colors[1] = secondBallSensor.getBallColor();
        thirdBallSensor.update();
        colors[2] = thirdBallSensor.getBallColor();

        sortLogic.update(colors, pattern, shootBtn.getState(), Timer.seconds(), rampBallColors);
        boolean sortingActive = sortLogic.isBusy();

        if (sortingActive) {
            SortServo1.setPosition(sortLogic.isOpenUpSort() ? sortOpenAngle : sortCloseAngle);
            SortServo2.setPosition(sortLogic.isOpenDownSort() ? sortOpenAngle : sortCloseAngle);

            double feedPower = sortLogic.isFeedOn() ? -1.0 : 0.0;
            IntakeL.setPower(feedPower);
            IntakeR.setPower(feedPower);

            //ShooterServo.setPosition(sortLogic.isShootOn() ? gateOpenAngle : gateCloseAngle);
            ShooterServo.setPosition(gateOpenAngle);
        } else {
            if(sortServoResetBtn.getState()) {
                SortServo1.setPosition(resetAngle);
                SortServo2.setPosition(resetAngle);
            }
            else {
                SortServo1.setPosition(sortServoBtnUp.getState() ? sortOpenAngle : sortCloseAngle);
                SortServo2.setPosition(sortServoBtnDown.getState() ? sortOpenAngle : sortCloseAngle);
            }

            double intakePower;
            if(intakeBtn.getState() && !reverseIntakeBtn.getState()) {
                intakePower = -1;
            }
            else if(!intakeBtn.getState() && reverseIntakeBtn.getState()) {
                intakePower = 0.5;
            }
            else {
                intakePower = 0;
            }
            IntakeL.setPower(intakePower);
            IntakeR.setPower(intakePower);

            ShooterServo.setPosition(shootBtn.getState() ? gateOpenAngle : gateCloseAngle);
        }

        telemetry.addData("Target RPM", RPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("ShooterR Position", ShooterR.getCurrentPosition());
        telemetry.addData("ShooterL Position", ShooterL.getCurrentPosition());
        telemetry.addData("Pattern", pattern);
        telemetry.addData("State", sortLogic.getState());
        telemetry.addData("Move", sortLogic.getMove());

        telemetry.addData("Colors", "%d %d", colors[0], colors[1]);
        telemetry.addData("Sorting", sortingActive);
        telemetry.addData("Sort Up/Down", "%b %b", sortLogic.isOpenUpSort(), sortLogic.isOpenDownSort());
        telemetry.addData("Sort feed/shoot", "%b %b", sortLogic.isFeedOn(), sortLogic.isShootOn());

        firstBallSensor.telemetry(telemetry, "UP");
        secondBallSensor.telemetry(telemetry, "DOWN");

        telemetry.update();
        telemetryM.update();
    }

    private double updateShooterPD(double targetRPM) {
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastTimeNs) / 1e9;
        if (dt <= 0) dt = 0.02;
        if (dt > 0.2) dt = 0.02;

        int pos = (Math.abs(ShooterR.getCurrentPosition()) + Math.abs(ShooterL.getCurrentPosition())) / 2;
        int dPos = pos - lastShooterPos;
        //dPos = -dPos;

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
    private double updateShooterPID(double targetRPM) {
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastTimeNs) / 1e9;
        if (dt <= 0) dt = 0.02;
        if (dt > 0.2) dt = 0.02;

        int pos = (Math.abs(ShooterR.getCurrentPosition()) + Math.abs(ShooterL.getCurrentPosition())) / 2;
        int dPos = pos - lastShooterPos;

        double rev = dPos / TICKS_PER_REV;
        currentRPM = (rev / dt) * 60.0;

        lastShooterPos = pos;
        lastTimeNs = nowNs;

        shooterPID.setPID(Kp, Ki, Kd);

        double pidOutput = shooterPID.calculate(currentRPM, targetRPM);
        double ff = targetRPM / maxShooterRPM;

        double power = pidOutput + ff;

        if (targetRPM <= 1) {
            power = 0.0;
            shooterPID.reset();
        }

        return Range.clip(power, 0.0, 1.0);
    }
}