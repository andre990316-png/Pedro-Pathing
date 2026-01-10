package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightAim;

@TeleOp(name = "Drivetrain2")
public class Drivetrain2 extends OpMode {

    public static final double TICKS_PER_REV = 28;

    // Drive + mechanisms
    private DcMotor MotorBackLeft;
    private DcMotor MotorFrontLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackRight;
    private DcMotor IntakeMotor;
    private DcMotor ShooterM1;
    private DcMotor ShooterM2;
    private DcMotor ShooterRotateMotor;
    private Servo ShooterS1;
    private Servo ShooterS2;

    // Vision + turret
    private LimelightAim limelight;
    private IMU imu;

    // ===== These USED to be locals in runOpMode(); now they MUST be fields =====
    private boolean Mode, CirclePrev, DpadUpPrev, DpadDownPrev, DpadLeftPrev, DpadRightPrev, Right_BumperPrev, Left_BumperPrev, GateOpen;
    private boolean Precision_mode_toggle, Precision_mode, IntakeToggle;

    private boolean AimTogglePrev = false;
    private boolean autoAimEnabled = true;

    private double pos;
    private double currentSensitivity;
    private double Sensitivity;
    private double XL, YL, XR, YR;
    private double TempMax1, TempMax2, MaxPower;
    private double shooterPower = 0.0;
    private boolean shooterEnabled = false;
    private long lastShooterTime;
    private int lastPos1, lastPos2;

    @Override
    public void init() {
        // ===== Hardware map =====
        MotorBackLeft = hardwareMap.get(DcMotor.class, "Motor Back Left");
        MotorFrontLeft = hardwareMap.get(DcMotor.class, "Motor Front Left");
        MotorFrontRight = hardwareMap.get(DcMotor.class, "Motor Front Right");
        MotorBackRight = hardwareMap.get(DcMotor.class, "Motor Back Right");

        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        ShooterM1 = hardwareMap.get(DcMotor.class, "Shooter M1");
        ShooterM2 = hardwareMap.get(DcMotor.class, "Shooter M2");
        ShooterS1 = hardwareMap.get(Servo.class, "Shooter S1");
        ShooterS2 = hardwareMap.get(Servo.class, "Shooter S2");
        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight = new LimelightAim(hardwareMap, imu, "Limelight");
        limelight.setPipeline("Blue");

        // Motor setup
        MotorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        MotorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        //ShooterM2.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== Init state (what you did before waitForStart) =====
        ShooterS1.setPosition(0.8);
        Sensitivity = 1.0;

        Mode = false;
        CirclePrev = false;
        DpadUpPrev = false;
        DpadDownPrev = false;
        Left_BumperPrev = false;
        Right_BumperPrev = false;
        GateOpen = false;

        Precision_mode_toggle = false;
        IntakeToggle = false;

        lastShooterTime = System.nanoTime();
        lastPos1 = ShooterM1.getCurrentPosition();
        lastPos2 = ShooterM2.getCurrentPosition();

        telemetry.addData("Initialize", "Completed");
        telemetry.update();
    }

    @Override
    public void start() {
        // This is where your code used to run RIGHT AFTER waitForStart()
        limelight.start(getRuntime());
    }

    @Override
    public void loop() {

        // ---------------------------
        // Shooter Motors' speed
        // ---------------------------
        long Shooter_now = System.nanoTime();
        double Shooter_dt = (Shooter_now - lastShooterTime) / 1e9;

        if (Shooter_dt <= 0) Shooter_dt = 0.02;

        int pos1 = ShooterM1.getCurrentPosition();
        int pos2 = ShooterM2.getCurrentPosition();

        int dPos1 = pos1 - lastPos1;
        int dPos2 = pos2 - lastPos2;

        double rev1 = dPos1 / TICKS_PER_REV;
        double rev2 = dPos2 / TICKS_PER_REV;

        double rpm1 = (rev1 / Shooter_dt) * 60.0;
        double rpm2 = (rev2 / Shooter_dt) * 60.0;

        lastPos1 = pos1;
        lastPos2 = pos2;
        lastShooterTime = Shooter_now;

        // ---------------------------
        // Shooter angle servo manual trim (gamepad2)
        // ---------------------------
        if (gamepad2.dpad_right && !DpadRightPrev) {
            pos += 0.05;
        }
        DpadRightPrev = gamepad2.dpad_right;
        if (gamepad2.dpad_left && !DpadLeftPrev) {
            pos -= 0.05;
        }
        DpadLeftPrev = gamepad2.dpad_left;
        pos = Range.clip(pos, 0.8, 1.0);
        ShooterS1.setPosition(pos);
        /*if (gamepad1.circle && !CirclePrev) {
            GateOpen = !GateOpen;
        }
        CirclePrev = gamepad1.circle;*/
        if (gamepad2.right_trigger > 0.03) {
            GateOpen = true;
        } else {
            GateOpen = false;
        }

        ShooterS2.setPosition(GateOpen? 0.55 : 0.8);

        // ---------------------------
        // Shooter motors (gamepad2 trigger)
        // ---------------------------

        if (gamepad2.dpad_up  && !DpadUpPrev) {
            shooterPower += 0.1;
        }
        DpadUpPrev = gamepad2.dpad_up;
        if (gamepad2.dpad_down && !DpadDownPrev) {
            shooterPower -= 0.1;
        }
        DpadDownPrev = gamepad2.dpad_down;
        shooterPower = Range.clip(shooterPower, 0.0, 1.0);
        ShooterM1.setPower(shooterPower);
        ShooterM2.setPower(-shooterPower);
        // ---------------------------
        // Intake toggle (gamepad1 left bumper)
        // ---------------------------
        if (gamepad1.left_bumper && !Left_BumperPrev) {
            IntakeToggle = !IntakeToggle;
        }
        IntakeMotor.setPower(IntakeToggle ? -1.0 : 0.0);
        Left_BumperPrev = gamepad1.left_bumper;

        // ---------------------------
        // Precision mode toggle + hold
        // ---------------------------
        /*if (gamepad1.circle && !CirclePrev) {
            Precision_mode_toggle = !Precision_mode_toggle;
        }
        CirclePrev = gamepad1.circle;

        Precision_mode = gamepad1.cross;

        currentSensitivity = (Precision_mode_toggle || Precision_mode) ? 0.3 : Sensitivity;

        // ---------------------------
        // Mode toggle (gamepad1 right bumper)
        // ---------------------------
        if (gamepad1.right_bumper && !Right_BumperPrev) {
            Mode = !Mode;
        }
        Right_BumperPrev = gamepad1.right_bumper;

        if (Mode) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // ---------------------------
        // Sensitivity adjust (dpad up/down)
        // ---------------------------
        if (gamepad1.dpad_up && !DpadUpPrev && !Precision_mode) {
            Sensitivity = Math.min(Math.max(Sensitivity + 0.1, 0.1), 1.0);
        }
        DpadUpPrev = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !DpadDownPrev && !Precision_mode) {
            Sensitivity = Math.min(Math.max(Sensitivity - 0.1, 0.1), 1.0);
        }
        DpadDownPrev = gamepad1.dpad_down;*/

        // ---------------------------
        // Drivetrain mixing
        // ---------------------------
        XL = gamepad1.left_stick_x;// * currentSensitivity;
        YL = -gamepad1.left_stick_y;// * currentSensitivity;
        XR = gamepad1.right_stick_x;// * currentSensitivity;
        YR = -gamepad1.right_stick_y;// * currentSensitivity; // unused but kept

        TempMax1 = Math.max(Math.abs(YL + XL + XR), Math.abs((YL - XL) - XR));
        TempMax2 = Math.max(Math.abs((YL - XL) + XR), Math.abs((YL + XL) - XR));
        MaxPower = Math.max(TempMax1, TempMax2);

        // Auto-straight assist
        /*if (Mode && Math.abs(XL) < 0.3 * currentSensitivity && Math.abs(YL) > 0.3 * currentSensitivity) {
            XL = 0;
        }*/

        if (MaxPower > 1) {
            MotorFrontLeft.setPower((YL + XL + XR) / MaxPower);
            MotorFrontRight.setPower(((YL - XL) - XR) / MaxPower);
            MotorBackLeft.setPower(((YL - XL) + XR) / MaxPower);
            MotorBackRight.setPower(((YL + XL) - XR) / MaxPower);
        } else {
            MotorFrontLeft.setPower(YL + XL + XR);
            MotorFrontRight.setPower((YL - XL) - XR);
            MotorBackLeft.setPower((YL - XL) + XR);
            MotorBackRight.setPower((YL + XL) - XR);
        }

        // ---------------------------
        // Auto-aim toggle (gamepad2 triangle)
        // ---------------------------
        /*if (gamepad1.triangle && !AimTogglePrev) {
            autoAimEnabled = !autoAimEnabled;
        }
        AimTogglePrev = gamepad1.triangle;*/
        if (gamepad2.left_trigger > 0.3) {
            autoAimEnabled = true;
        } else {
            autoAimEnabled = false;
        }

        // ---------------------------
        // Update IMU yaw
        // ---------------------------
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // ---------------------------
        // Turret control
        //  - Auto-aim PD when enabled
        //  - Manual control otherwise
        // ---------------------------
        double turretPower = 0;

        if (autoAimEnabled) {
            turretPower = limelight.update(getRuntime(), telemetry);
        } else {
            // Manual turret rotate using gamepad2 right stick X
            double manual = gamepad2.right_stick_x;
            if (Math.abs(manual) > 0.08) {
                turretPower = Range.clip(manual * 0.4, -0.4, 0.4);
            } else {
                turretPower = 0;
            }
            // Reset PD history so it doesn't "jump" when you re-enable auto
            limelight.resetHistory(getRuntime());
            telemetry.addData("AutoAim", "OFF");
            telemetry.addData("Turret Manual", turretPower);
        }
        ShooterRotateMotor.setPower(turretPower);

        // Shooter Input Sum
        double shooterPowerSum = ShooterM1.getPower() + Math.abs(ShooterM2.getPower());

        // ---------------------------
        // Telemetry
        // ---------------------------
        telemetry.addData("Motor 1 Output", MotorFrontLeft.getPower());
        telemetry.addData("Motor 2 Output", MotorFrontRight.getPower());
        telemetry.addData("Motor 3 Output", MotorBackLeft.getPower());
        telemetry.addData("Motor 4 Output", MotorBackRight.getPower());

        telemetry.addData("Shooter Servo", ShooterS1.getPosition());
        telemetry.addData("Shooter Servo2", ShooterS2.getPosition());
        telemetry.addData("Shooter M1 Input", ShooterM1.getPower());
        telemetry.addData("Shooter M2 Input", ShooterM2.getPower());
        telemetry.addData("Shooter Input Sum", shooterPowerSum);
        telemetry.addData("Shooter M1 RPM", rpm1);
        telemetry.addData("Shooter M2 RPM", rpm2);
        telemetry.addData("Intake Power", IntakeMotor.getPower());
        //telemetry.addData("Sensitivity Output", currentSensitivity);
        //telemetry.addData("Mode", Mode);
        //telemetry.addData("Precision toggle", Precision_mode_toggle);
        //telemetry.addData("Precision hold", Precision_mode);
        //telemetry.addData("Sensitivity", Sensitivity);
        telemetry.update();
    }
}