package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Drivetrain")
public class Drivetrain extends LinearOpMode {

    // Drive + mechanisms
    private DcMotor MotorBackLeft;
    private DcMotor MotorFrontLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackRight;
    private DcMotor IntakeMotor;
    private DcMotor ShooterM1; // right
    private DcMotor ShooterM2; // left
    private Servo ShooterS1;   // angle/position servo

    // Vision + turret
    private Limelight3A limelight;
    private CRServo limelightServo; // turret rotate CRServo
    private IMU imu;

    // PD constants for turret
    private double Kp = 0.01;
    private double Kd = 0.003;
    private double deadband = 1.0; // degrees
    private double maxTurretPower = 0.30;

    // PD state
    private double lastTx = 0;
    private double lastTime = 0;

    @Override
    public void runOpMode() {

        // ===== Variables from your original TeleOp =====
        boolean Mode;
        boolean CirclePrev;
        boolean DpadUpPrev;
        boolean DpadDownPrev;
        boolean Right_BumperPrev;
        boolean Left_BumperPrev;
        boolean Precision_mode_toggle;
        boolean Precision_mode;
        boolean IntakeToggle;

        double pos;
        double currentSensitivity;
        double Sensitivity;
        double XL, YL, XR, YR;
        double TempMax1, TempMax2, MaxPower;

        // ===== New: auto-aim toggle =====
        boolean AimTogglePrev = false;
        boolean autoAimEnabled = true;

        // ===== Hardware map =====
        MotorBackLeft = hardwareMap.get(DcMotor.class, "Motor Back Left");
        MotorFrontLeft = hardwareMap.get(DcMotor.class, "Motor Front Left");
        MotorFrontRight = hardwareMap.get(DcMotor.class, "Motor Front Right");
        MotorBackRight = hardwareMap.get(DcMotor.class, "Motor Back Right");

        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        ShooterM1 = hardwareMap.get(DcMotor.class, "Shooter M1");
        ShooterM2 = hardwareMap.get(DcMotor.class, "Shooter M2");
        ShooterS1 = hardwareMap.get(Servo.class, "Shooter S1");

        // Vision + turret
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(3);
        limelightServo = hardwareMap.get(CRServo.class, "ShooterRotateServo");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // Motor directions
        MotorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        MotorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        // ===== Init state =====
        pos = ShooterS1.getPosition();
        Sensitivity = 1.0;

        Mode = false;
        CirclePrev = false;
        DpadUpPrev = false;
        DpadDownPrev = false;
        Left_BumperPrev = false;
        Right_BumperPrev = false;

        Precision_mode_toggle = false;
        IntakeToggle = false;

        telemetry.addData("Initialize", "Completed");
        telemetry.update();

        waitForStart();

        // Start Limelight after start
        limelight.start();
        lastTime = getRuntime();
        lastTx = 0;

        // ===== Main loop =====
        while (opModeIsActive()) {

            // ---------------------------
            // Shooter angle servo manual trim (gamepad2)
            // ---------------------------
            if (gamepad2.dpad_right) {
                pos += 0.001;
                if (pos > 1.0) pos = 1.0;
                ShooterS1.setPosition(pos);
            }
            if (gamepad2.dpad_left) {
                pos -= 0.001;
                if (pos < 0.0) pos = 0.0;
                ShooterS1.setPosition(pos);
            }

            // ---------------------------
            // Shooter motors (gamepad2 trigger)
            // ---------------------------
            if (gamepad2.right_trigger > 0) {
                ShooterM1.setPower(gamepad2.right_trigger);
                ShooterM2.setPower(-gamepad2.right_trigger);
            } else {
                ShooterM1.setPower(0);
                ShooterM2.setPower(0);
            }

            // ---------------------------
            // Intake toggle (gamepad1 left bumper)
            // ---------------------------
            if (gamepad1.left_bumper && !Left_BumperPrev) {
                IntakeToggle = !IntakeToggle;
            }
            IntakeMotor.setPower(IntakeToggle ? 1.0 : 0.0);
            Left_BumperPrev = gamepad1.left_bumper;

            // ---------------------------
            // Precision mode toggle + hold
            // ---------------------------
            if (gamepad1.circle && !CirclePrev) {
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
            DpadDownPrev = gamepad1.dpad_down;

            // ---------------------------
            // Drivetrain mixing
            // ---------------------------
            XL = gamepad1.left_stick_x * currentSensitivity;
            YL = -gamepad1.left_stick_y * currentSensitivity;
            XR = gamepad1.right_stick_x * currentSensitivity;
            YR = -gamepad1.right_stick_y * currentSensitivity; // unused but kept

            TempMax1 = Math.max(Math.abs(YL + XL + XR), Math.abs((YL - XL) - XR));
            TempMax2 = Math.max(Math.abs((YL - XL) + XR), Math.abs((YL + XL) - XR));
            MaxPower = Math.max(TempMax1, TempMax2);

            // Auto-straight assist
            if (Mode && Math.abs(XL) < 0.3 * currentSensitivity && Math.abs(YL) > 0.3 * currentSensitivity) {
                XL = 0;
            }

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
            if (gamepad2.triangle && !AimTogglePrev) {
                autoAimEnabled = !autoAimEnabled;
            }
            AimTogglePrev = gamepad2.triangle;

            // ---------------------------
            // Update IMU yaw for Limelight
            // ---------------------------
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());

            // ---------------------------
            // Turret control
            //  - Auto-aim PD when enabled
            //  - Manual control otherwise
            // ---------------------------
            double turretPower = 0;

            if (autoAimEnabled) {
                LLResult llResult = limelight.getLatestResult();

                if (llResult != null && llResult.isValid()) {
                    double tx = llResult.getTx();
                    double error = -tx;

                    double now = getRuntime();
                    double dt = now - lastTime;
                    if (dt <= 0) dt = 0.02;

                    double dTx = (tx - lastTx) / dt;

                    if (Math.abs(tx) <= deadband) {
                        turretPower = 0;
                    } else {
                        turretPower = Kp * error - Kd * dTx;
                    }

                    turretPower = Range.clip(turretPower, -maxTurretPower, maxTurretPower);

                    lastTx = tx;
                    lastTime = now;

                    telemetry.addData("AutoAim", "ON");
                    telemetry.addData("tx", tx);
                    telemetry.addData("dTx", dTx);
                    telemetry.addData("Turret PD", turretPower);
                } else {
                    // No valid tag -> stop turret for safety
                    turretPower = 0;
                    telemetry.addData("AutoAim", "ON (no valid tag)");
                }

            } else {
                // Manual turret rotate using gamepad2 right stick X
                double manual = gamepad2.right_stick_x;
                if (Math.abs(manual) > 0.08) {
                    turretPower = Range.clip(manual * 0.4, -0.4, 0.4);
                } else {
                    turretPower = 0;
                }

                // Reset PD history so it doesn't "jump" when you re-enable auto
                lastTx = 0;
                lastTime = getRuntime();

                telemetry.addData("AutoAim", "OFF");
                telemetry.addData("Turret Manual", turretPower);
            }

            limelightServo.setPower(turretPower);

            // ---------------------------
            // Telemetry
            // ---------------------------
            telemetry.addData("Motor FL", MotorFrontLeft.getPower());
            telemetry.addData("Motor FR", MotorFrontRight.getPower());
            telemetry.addData("Motor BL", MotorBackLeft.getPower());
            telemetry.addData("Motor BR", MotorBackRight.getPower());

            telemetry.addData("Shooter S1 Pos", ShooterS1.getPosition());
            telemetry.addData("Shooter M1", ShooterM1.getPower());
            telemetry.addData("Shooter M2", ShooterM2.getPower());

            telemetry.addData("Intake", IntakeMotor.getPower());
            telemetry.addData("Sensitivity", currentSensitivity);
            telemetry.addData("Precision Toggle", Precision_mode_toggle);
            telemetry.addData("Precision Hold", Precision_mode);
            telemetry.addData("Mode", Mode);

            telemetry.addData("IMU Yaw", orientation.getYaw());
            telemetry.update();
        }
    }
}