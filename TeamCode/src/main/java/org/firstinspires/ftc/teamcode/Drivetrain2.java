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
import org.firstinspires.ftc.teamcode.mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightAim;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Drivetrain2")
public class Drivetrain2 extends OpMode {


    public static AutoShooting lookupA(double ta) {
        if (ta >= 2.37) return new AutoShooting(3200, 0.90);
        if (ta >= 1.10) return new AutoShooting(3900, 0.94);
        if (ta >= 0.70) return new AutoShooting(4400, 1.00);
        return new AutoShooting(4800, 1.00);
    }

    public static AutoShooting lookupB(double ta) {
        if (ta >= 0.38) return new AutoShooting(4800, 1.00);
        if (ta >= 0.32) return new AutoShooting(4900, 1.00);
        if (ta >= 0.29) return new AutoShooting(5200, 1.00);
        return new AutoShooting(5400, 1.00);
    }


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
    private boolean DpadUpPrev, DpadDownPrev, DpadLeftPrev, DpadRightPrev, Right_BumperPrev, Left_BumperPrev, GateOpen;
    private boolean Precision_mode_toggle, Precision_mode, IntakeToggle;
    private boolean autoAimEnabled = true;
    private boolean flywheelAutoMode = false;
    private boolean flywheelTogglePrev = false;

    private double pos;
    private double currentSensitivity;
    private double Sensitivity;
    private double XL, YL, XR, YR;
    private double TempMax1, TempMax2, MaxPower;
    private double shooterPower = 0.0;
    private long lastShooterTime;
    private int lastPos1;

    // Linear Flywheels
    double targetRPM = 0;

    public static final double kP = 0.39;   // tune this
    public static final double kI = 0.000010;  // optional
    public static final double kD = 0.00002;   // optional

    double rpmErrorSum = 0;
    double lastError = 0;

    private FlywheelLogic shooter = new FlywheelLogic();



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
        ShooterM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== Init state (what you did before waitForStart) =====
        ShooterS1.setPosition(0.84);

        DpadUpPrev = false;
        DpadDownPrev = false;
        Left_BumperPrev = false;
        Right_BumperPrev = false;
        GateOpen = false;

        Precision_mode_toggle = false;
        IntakeToggle = false;

        shooter.setTargetRPM(0);
        flywheelAutoMode = false;
        rpmErrorSum = 0;
        lastError = 0;

        lastShooterTime = System.nanoTime();
        lastPos1 = ShooterM1.getCurrentPosition();

        telemetry.addData("Initialize", "Completed");
        telemetry.update();

        shooter.init(hardwareMap);
    }

    @Override
    public void start() {
        // This is where your code used to run RIGHT AFTER waitForStart()
        limelight.start(getRuntime());
    }

    @Override
    public void loop() {

        /**long Shooter_now = System.nanoTime();
        double Shooter_dt = (Shooter_now - lastShooterTime) / 1e9;

        if (Shooter_dt <= 0) Shooter_dt = 0.02;

        int pos1 = ShooterM1.getCurrentPosition();

        int dPos1 = pos1 - lastPos1;
        dPos1 = -dPos1;

        double rev1 = dPos1 / TICKS_PER_REV;

        double rpm = (rev1 / Shooter_dt) * 60.0;

        lastPos1 = pos1;
        lastShooterTime = Shooter_now;*/
        shooter.update();
        // ---------------------------
        // Shooter angle servo manual trim (gamepad2)
        // ---------------------------
        if (gamepad2.dpad_right && !DpadRightPrev) {
            pos += 0.02;
        }
        DpadRightPrev = gamepad2.dpad_right;
        if (gamepad2.dpad_left && !DpadLeftPrev) {
            pos -= 0.02;
        }
        DpadLeftPrev = gamepad2.dpad_left;
        pos = Range.clip(pos, 0.84, 1.0);
        ShooterS1.setPosition(pos);

        GateOpen = gamepad2.right_trigger > 0.03;
        ShooterS2.setPosition(GateOpen? 0 : 0.2);

        // Read ta only if valid

        double ta = 0;
        boolean llValid = false;
        LLResult ll = limelight.limelight.getLatestResult();
        if (ll != null && ll.isValid()) {
            llValid = true;
            ta = ll.getTa();
        }

        if (gamepad2.right_bumper && !flywheelTogglePrev) {
            flywheelAutoMode = !flywheelAutoMode;
            shooter.setTargetRPM(0);
        }
        flywheelTogglePrev = gamepad2.right_bumper;

        if (flywheelAutoMode) {
            if (llValid) {
                AutoShooting shot = (ta >= 0.7) ? lookupA(ta) : lookupB(ta);
                shooter.setTargetRPM(shot.rpm);
                shooter.setHoodPosition(shot.hood);
            } else {
                shooter.setTargetRPM(0);
            }
        } else {
            if (gamepad2.dpad_up && !DpadUpPrev)   shooter.setTargetRPM(shooter.getTargetRPM()+500);
            if (gamepad2.dpad_down && !DpadDownPrev) shooter.setTargetRPM(shooter.getTargetRPM()-500);
            DpadUpPrev = gamepad2.dpad_up;
            DpadDownPrev = gamepad2.dpad_down;
        }
        /**targetRPM = Range.clip(targetRPM, 0, 6000);

        double error = targetRPM - rpm;
        double dError = (error - lastError) / Shooter_dt;

        double pdPower = kP * error + kD * dError;

        if (targetRPM <= 0) pdPower = 0;

        shooterPower = Range.clip(pdPower, 0.0, 1.0);

        ShooterM1.setPower(shooterPower);
        ShooterM2.setPower(-shooterPower);

        lastError = error;*/



        // ---------------------------
        // Intake toggle (gamepad1 left bumper)
        // ---------------------------
        if (gamepad1.left_bumper && !Left_BumperPrev) {
            IntakeToggle = !IntakeToggle;
        }
        IntakeMotor.setPower(IntakeToggle ? -1.0 : 0.0);
        Left_BumperPrev = gamepad1.left_bumper;


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
        autoAimEnabled = gamepad2.left_trigger > 0.3;

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
        telemetry.addData("Intake Power", IntakeMotor.getPower());

        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("RPM", shooter.getFlywheelRpm());
        telemetry.addData("Flywheel Error", shooter.getError());
        telemetry.addData("Flywheel Power", shooterPower);


        telemetry.update();
    }


}