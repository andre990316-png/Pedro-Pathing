package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.teamcode.Mechanisms.ButtonLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.LimelightAim;
import org.firstinspires.ftc.teamcode.Mechanisms.AutoShooting;
import org.firstinspires.ftc.teamcode.Data.FlywheelAndHoodData;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
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
    private Limelight3A limelight;
    private IMU imu;
    private LimelightAim autoAim = new LimelightAim();
    private boolean precisionMode;
    private double currentSensitivity;
    private double Sensitivity;
    private AutoShooting shot;
    private double pos;
    private double XL, YL, XR, YR;
    private double TempMax1, TempMax2, MaxPower;
    private double shooterPower = 0.0;

    //Battery Voltage
    private VoltageSensor battery;

    // Linear Flywheels
    double targetRPM = 0;

    private FlywheelLogic shooter = new FlywheelLogic();
    private ButtonLogic hoodUpBtn      = new ButtonLogic(ButtonLogic.Mode.PULSE, false);  // dpad_right
    private ButtonLogic hoodDownBtn    = new ButtonLogic(ButtonLogic.Mode.PULSE, false);  // dpad_left
    private ButtonLogic rpmUpBtn       = new ButtonLogic(ButtonLogic.Mode.PULSE, false);  // dpad_up
    private ButtonLogic rpmDownBtn     = new ButtonLogic(ButtonLogic.Mode.PULSE, false);  // dpad_down

    //private ButtonLogic gateHoldBtn    = new ButtonLogic(ButtonLogic.Mode.HOLD, false);   // right_trigger
    private ButtonLogic shoot3Btn    = new ButtonLogic(ButtonLogic.Mode.HOLD, false);   // right_trigger
    private ButtonLogic autoAimHoldBtn = new ButtonLogic(ButtonLogic.Mode.HOLD, false);   // left_trigger

    private ButtonLogic intakeToggleBtn    = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false); // gamepad1.left_bumper
    private ButtonLogic autoFlywheelAndHoodToggleBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false); // gamepad2.right_bumper

    private ButtonLogic precisionModeToggleBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);//gamepad1.right_bumper
    private ButtonLogic precisionModeHoldBtn = new ButtonLogic(ButtonLogic.Mode.HOLD, false);//gamepad1.right_trigger
    private ButtonLogic sensitivityUpBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);//gamepad1.dpad_up
    private ButtonLogic sensitivityDownBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);//gamepad1.dpad_down

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
        battery = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));

        // Motor setup
        MotorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        MotorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        //ShooterM2.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterS1.setPosition(0.84);

        shooter.setTargetRPM(0);

        currentSensitivity = 1.0;

        shooter.init(hardwareMap);

        telemetry.addData("Initialize", "Completed");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        autoAim.resetHistory(getRuntime());
    }


    //====================
    //Main Loop
    //====================
    @Override
    public void loop() {
        shooter.update();

        //Update Buttons
        hoodUpBtn.update(gamepad2.dpad_right);
        hoodDownBtn.update(gamepad2.dpad_left);
        rpmUpBtn.update(gamepad2.dpad_up);
        rpmDownBtn.update(gamepad2.dpad_down);
        shoot3Btn.update(gamepad2.right_trigger > 0.3);
        //gateHoldBtn.update(gamepad2.right_trigger > 0.03);
        autoAimHoldBtn.update(gamepad2.left_trigger > 0.3);

        intakeToggleBtn.update(gamepad1.left_bumper);
        autoFlywheelAndHoodToggleBtn.update(gamepad2.right_bumper);

        precisionModeHoldBtn.update(gamepad1.right_trigger > 0.03);
        //precisionModeToggleBtn.update(gamepad1.right_bumper);
        sensitivityDownBtn.update(gamepad1.dpad_down);
        sensitivityUpBtn.update(gamepad1.dpad_up);

        //ShooterS2.setPosition(gateHoldBtn.getState()? 0 : 0.2);
        if (shoot3Btn.getState() && !shooter.isBusy()) {
            shooter.fireShots(1);
        }

        IntakeMotor.setPower(intakeToggleBtn.getState() ? -1.0 : 0.0);

//        if (precisionModeToggleBtn.getState()) {
//            precisionMode = !precisionMode;
//        }

        precisionMode = precisionModeHoldBtn.getState();

        //currentSensitivity = (precisionModeToggleBtn.getState() || precisionMode) ? 0.3 : Sensitivity;

        if (sensitivityUpBtn.getState() && precisionMode) {
            currentSensitivity = Math.min(Math.max(Sensitivity + 0.1, 0.1), 1.0);
        }

        if (sensitivityDownBtn.getState() && precisionMode) {
            currentSensitivity = Math.min(Math.max(Sensitivity - 0.1, 0.1), 1.0);
        }

        XL = gamepad1.left_stick_x * currentSensitivity;
        YL = -gamepad1.left_stick_y * currentSensitivity;
        XR = gamepad1.right_stick_x * currentSensitivity;
        YR = -gamepad1.right_stick_y * currentSensitivity;

        TempMax1 = Math.max(Math.abs(YL + XL + XR), Math.abs((YL - XL) - XR));
        TempMax2 = Math.max(Math.abs((YL - XL) + XR), Math.abs((YL + XL) - XR));
        MaxPower = Math.max(TempMax1, TempMax2);

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

        double yaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(yaw);

        double turretPower;
        LLResult ll = limelight.getLatestResult();
        if (autoAimHoldBtn.getState()) {
            turretPower = autoAim.update(getRuntime(), ll, telemetry);
        } else {
            double manual = gamepad2.right_stick_x;
            turretPower = (Math.abs(manual) > 0.08) ? Range.clip(manual * 0.4, -0.4, 0.4) : 0;
            autoAim.resetHistory(getRuntime());
        }
        ShooterRotateMotor.setPower(turretPower);

        double ta = 0;
        boolean llValid = false;
        if (ll != null && ll.isValid()) {
            llValid = true;
            ta = ll.getTa();
        }

        if(autoFlywheelAndHoodToggleBtn.justPressed()) shooter.setTargetRPM(0);
        if (autoFlywheelAndHoodToggleBtn.getState()) {
            if(llValid) {
                shot = (ta >= 0.7) ? FlywheelAndHoodData.lookupA(ta) : FlywheelAndHoodData.lookupB(ta);
                shooter.setTargetRPM(shot.rpm);
                shooter.setHoodPosition(shot.hood);
            } else {
                shooter.setTargetRPM(shot.rpm);
                shooter.setHoodPosition(shot.hood);
            }
        } else {
            if (rpmUpBtn.getState()) shooter.setTargetRPM(shooter.getTargetRPM() + 500);
            if (rpmDownBtn.getState()) shooter.setTargetRPM(shooter.getTargetRPM() - 500);
            if (hoodUpBtn.getState()) pos += 0.02;
            if (hoodDownBtn.getState()) pos -= 0.02;
            pos = Range.clip(pos, 0.84, 1.0);
            ShooterS1.setPosition(pos);
        }

        // Telemetry
        telemetry.addLine("In-Game");
        telemetry.addLine("                                  ");
        telemetry.addData("Target RPM ", shooter.getTargetRPM());
        telemetry.addData("RPM ", shooter.getFlywheelRpm());
        telemetry.addLine("                                  ");
        telemetry.addData("Shooter Servo ", ShooterS1.getPosition());
        telemetry.addData("Shooter Servo2 ", ShooterS2.getPosition());
        telemetry.addLine("                                  ");
        telemetry.addData("Sensitivity", Sensitivity);
        telemetry.addData("Precision Mode Toggle", precisionModeToggleBtn.getState());
        telemetry.addData("Precision Mode Hold", precisionModeHoldBtn.getState());

        telemetry.addLine("                                  ");
        telemetry.addLine("                                  ");
        telemetry.addLine("                                  ");

        telemetry.addLine("Debug");
        telemetry.addLine("                                  ");
        telemetry.addData("Motor 1 Power","%.3f",MotorFrontLeft.getPower());
        telemetry.addData("Motor 2 Power","%.3f", MotorFrontRight.getPower());
        telemetry.addData("Motor 3 Power","%.3f", MotorBackLeft.getPower());
        telemetry.addData("Motor 4 Power","%.3f", MotorBackRight.getPower());
        telemetry.addLine("                                  ");
        telemetry.addData("Shooter Servo", ShooterS1.getPosition());
        telemetry.addData("Shooter Servo2", ShooterS2.getPosition());
        telemetry.addData("Shooter M1 Input", ShooterM1.getPower());
        telemetry.addData("Shooter M2 Input", ShooterM2.getPower());
        telemetry.addData("Intake Power", IntakeMotor.getPower());
        telemetry.addLine("                                  ");
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("RPM", shooter.getFlywheelRpm());
        telemetry.addData("Flywheel Error", shooter.getError());
        telemetry.addData("Flywheel Power", shooterPower);
        telemetry.addData("Battery Voltage", "%.2f V", battery.getVoltage());

        telemetry.update();
    }
}