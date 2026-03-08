package org.firstinspires.ftc.teamcode.Tests;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

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
    private NormalizedColorSensor ColorSensorUp1;
    private NormalizedColorSensor ColorSensorUp2;
    private NormalizedColorSensor ColorSensorDown1;
    private NormalizedColorSensor ColorSensorDown2;
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

    @Override
    public void init() {
        ShooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        ShooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        IntakeR  = hardwareMap.get(DcMotorEx.class, "intakeR");
        IntakeL  = hardwareMap.get(DcMotorEx.class, "intakeL");
        ShooterServo = hardwareMap.get(Servo.class, "servo");
        SortServo1 = hardwareMap.get(Servo.class, "sortServo1");
        SortServo2 = hardwareMap.get(Servo.class, "sortServo2");
        ColorSensorUp1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensorUp1");
        ColorSensorUp2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensorUp2");
        ColorSensorUp1.setGain(16.0f);
        ColorSensorUp2.setGain(16.0f);
        IntakeR.setDirection(DcMotor.Direction.REVERSE);

        ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterServo.setPosition(gateCloseAngle);
        SortServo1.setPosition(0);
        SortServo2.setPosition(0);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        lastShooterPos = (ShooterR.getCurrentPosition() + ShooterR.getCurrentPosition()) / 2;
        lastTimeNs = System.nanoTime();
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
        if(sortServoBtnUp.getState())
            SortServo1.setPosition(sortOpenAngle);
        else
            SortServo1.setPosition(sortCloseAngle);
        if(sortServoBtnDown.getState())
            SortServo2.setPosition(sortOpenAngle);
        else
            SortServo2.setPosition(sortCloseAngle);

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

        NormalizedRGBA newColors1 = ColorSensorUp1.getNormalizedColors();
        NormalizedRGBA newColors2 = ColorSensorUp2.getNormalizedColors();
        int c1 = getColorHSV(newColors1);
        int c2 = getColorHSV(newColors2);
        int ballColor = fuseBallColor(c1, c2);
        colors[0] = ballColor;

        float[] hsv1 = new float[3];
        Color.colorToHSV(newColors1.toColor(), hsv1);
        float[] hsv2 = new float[3];
        Color.colorToHSV(newColors2.toColor(), hsv2);

        telemetry.addData("Target RPM", RPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Gate Servo Open Angle", gateOpenAngle);
        telemetry.addData("Sort Servo Open Angle", sortOpenAngle);
        telemetry.addData("Slot 1 color", colors[0]);
        telemetry.addLine("Color 1")
                .addData("Red", "%.3f", newColors1.red)
                .addData("Green", "%.3f", newColors1.green)
                .addData("Blue", "%.3f", newColors1.blue);
        telemetry.addLine("Color 2")
                .addData("Red", "%.3f", newColors2.red)
                .addData("Green", "%.3f", newColors2.green)
                .addData("Blue", "%.3f", newColors2.blue);
        telemetry.addData("HSV1", "H=%.1f S=%.2f V=%.2f", hsv1[0], hsv1[1], hsv1[2]);
        telemetry.addData("HSV2", "H=%.1f S=%.2f V=%.2f", hsv2[0], hsv2[1], hsv2[2]);
        int r255 = (int) Range.clip(newColors1.red   * 255.0, 0.0, 255.0);
        int g255 = (int) Range.clip(newColors1.green * 255.0, 0.0, 255.0);
        int b255 = (int) Range.clip(newColors1.blue  * 255.0, 0.0, 255.0);

        telemetry.addData("RGB1_255", "%d %d %d", r255, g255, b255);
        int r255_2 = (int) Range.clip(newColors2.red   * 255.0, 0.0, 255.0);
        int g255_2 = (int) Range.clip(newColors2.green * 255.0, 0.0, 255.0);
        int b255_2 = (int) Range.clip(newColors2.blue  * 255.0, 0.0, 255.0);

        telemetry.addData("RGB2_255", "%d %d %d", r255_2, g255_2, b255_2);
        float[] hsv1255 = new float[3];
        Color.RGBToHSV(r255, g255, b255, hsv1255);
        telemetry.addData("hsv1255", "H=%.1f S=%.2f V=%.2f", hsv1255[0], hsv1255[1], hsv1255[2]);
        float[] hsv2255 = new float[3];
        Color.RGBToHSV(r255_2, g255_2, b255_2, hsv2255);
        telemetry.addData("hsv2255", "H=%.1f S=%.2f V=%.2f", hsv2255[0], hsv2255[1], hsv2255[2]);
        telemetry.addData("Error", (targetRPM - currentRPM));
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

        int pos = (ShooterR.getCurrentPosition() + ShooterR.getCurrentPosition()) / 2;
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
    public static int getColor(NormalizedRGBA color){
        float[] hsv = new float[3];
        Color.colorToHSV(color.toColor(), hsv);
        if(hsv[1] < 0.2){//if saturation too low, probably nothing
            return 0;
        }
        if((color.green + color.blue > 0.02) && (color.green < color.blue)){//main defining factor between green and purple is the G value
            return 1;
        }
        return 2;
    }
    public static int getColorHSV(NormalizedRGBA c) {
        float[] hsv = new float[3];
        Color.colorToHSV(c.toColor(), hsv);

        float H = hsv[0];
        float S = hsv[1];

        double intensity = c.red + c.green + c.blue;

        if (intensity < 0.02 || S < 0.15) return 0;

        if (H >= 140 && H <= 170) return 1;

        if (H >= 180 && H <= 240) return 2;

        return 0;
    }
    public static int fuseBallColor(int c1, int c2) {
        if (c1 == 0 && c2 == 0) return 0;
        if (c1 == c2) return c1;
        if (c1 == 0) return c2;
        if (c2 == 0) return c1;
        return 0;
    }
}