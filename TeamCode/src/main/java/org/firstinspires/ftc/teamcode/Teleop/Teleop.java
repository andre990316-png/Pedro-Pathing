package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Autonomous.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Constants;
import org.firstinspires.ftc.teamcode.Data.AllianceData;
import org.firstinspires.ftc.teamcode.Data.Auto_lastPose;
import org.firstinspires.ftc.teamcode.Mechanisms.ButtonLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.LimelightAim;
import org.firstinspires.ftc.teamcode.Mechanisms.AutoShooting;
import org.firstinspires.ftc.teamcode.Data.FlywheelAndHoodData;
import org.firstinspires.ftc.teamcode.Mechanisms.TeleopGate;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
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
    private TeleopGate teleopGate;
    // Vision + turret
    private Limelight3A limelight;
    private IMU imu;
    private LimelightAim autoAim = new LimelightAim();
    private boolean precisionMode;
    private boolean poseSnapped = false;
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
    //elapsed time
    private ElapsedTime allianceBannerTimer = new ElapsedTime();
    private boolean showAllianceBanner = true;


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
    private ButtonLogic autoSort = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic precisionModeToggleBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);//gamepad1.right_bumper
    private ButtonLogic precisionModeHoldBtn = new ButtonLogic(ButtonLogic.Mode.HOLD, false);//gamepad1.right_trigger
    private ButtonLogic sensitivityUpBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);//gamepad1.dpad_up
    private ButtonLogic sensitivityDownBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);//gamepad1.dpad_down
    private Follower follower;

    private final Pose topLeftStartPose = new Pose(20, 123, Math.toRadians(143));
    private final Pose bottomLeftStartPose = new Pose(48, 10, Math.toRadians(90));
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
        teleopGate = new TeleopGate(ShooterS2);
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
        Sensitivity = 1.0;

        shooter.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        if (Auto_lastPose.currentPose != null){
            follower.setStartingPose(Auto_lastPose.currentPose);
            telemetry.addLine("Starting Position = Auto_lastPose");
        }else {
            follower.setStartingPose(topLeftStartPose);
            telemetry.addLine("Starting Position = topLeftStartingPose");
        }

        telemetry.addData("Initialize", "Completed");
        telemetry.update();
    }

    @Override
    public void init_loop() {

        if (gamepad1.dpad_left) {
            AllianceData.selectedAlliance = AllianceData.Alliance.RED;
        }
        else if (gamepad1.dpad_right) {
            AllianceData.selectedAlliance = AllianceData.Alliance.BLUE;
        }

        telemetry.addLine("=== ALLIANCE SELECT ===");
        telemetry.addData("Alliance", AllianceData.selectedAlliance);
        telemetry.addLine("D-pad LEFT = RED");
        telemetry.addLine("D-pad RIGHT = BLUE");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        autoAim.resetHistory(getRuntime());

        if (AllianceData.isRed()) {
            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Red"));
        } else {
            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));
        }
        allianceBannerTimer.reset();
        showAllianceBanner = true;

        limelight.start();
        autoAim.resetHistory(getRuntime());
    }



    //====================
    //Main Loop
    //====================
    @Override
    public void loop() {
        shooter.update();
        follower.update();
        //teleopGate.update(shooter.getTargetRPM(), gamepad2.right_trigger > 0.3);
        LLResult ll = limelight.getLatestResult();

//        if (!poseSnapped && ll != null && ll.isValid()) {
//            // Example: botpose_MT2 gives a Pose3D-like object in FTC SDK
//            double x = (ll.getBotpose().getPosition().x - 1.83) * 39.3442622951;
//            double y = (ll.getBotpose().getPosition().y + 1.83) * 39.3442622951;
//            double yawDeg = ll.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
//
//            // If Limelight is in meters and Pedro is in inches, convert:
//            // x *= 39.3701; y *= 39.3701;
//
//
//            Pose snapped = new Pose(x, y, Math.toRadians(yawDeg));
//            follower.setPose(snapped);      // or follower.setStartingPose(snapped) depending on your Pedro version
//            poseSnapped = true;
//        }
        //Update Buttons
        hoodUpBtn.update(gamepad2.dpad_right);
        hoodDownBtn.update(gamepad2.dpad_left);
        rpmUpBtn.update(gamepad2.dpad_up);
        rpmDownBtn.update(gamepad2.dpad_down);
        shoot3Btn.update(gamepad2.right_trigger > 0.3);
        //gateHoldBtn.update(gamepad2.right_trigger > 0.03);
        autoAimHoldBtn.update(gamepad2.left_trigger > 0.3);
        autoSort.update(gamepad2.left_bumper);
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

        if (showAllianceBanner) {
            if (allianceBannerTimer.seconds() < .67) {
                telemetry.addLine("=== ALLIANCE LOCKED ===");
                telemetry.addData("Alliance", AllianceData.selectedAlliance);
                telemetry.update();
                return; // optional — remove if you want normal telemetry underneath
            } else {
                showAllianceBanner = false;
            }
        }

        IntakeMotor.setPower(intakeToggleBtn.getState() ? -1.0 : 0.0);

//        if (precisionModeToggleBtn.getState()) {
//            precisionMode = !precisionMode;
//        }

        precisionMode = precisionModeHoldBtn.getState();

        //currentSensitivity = (precisionModeToggleBtn.getState() || precisionMode) ? 0.3 : Sensitivity;

        if (sensitivityUpBtn.getState()) {
            Sensitivity = Range.clip(Sensitivity + 0.1, 0, 1);
            currentSensitivity = Sensitivity;
        }

        if (sensitivityDownBtn.getState()) {
            Sensitivity = Range.clip(Sensitivity - 0.1, 0, 1);
            currentSensitivity = Sensitivity;
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

        Pose robotPose = follower.getPose();
        double dx = AllianceData.getGoalPose().getX() - robotPose.getX();
        double dy = AllianceData.getGoalPose().getY() - robotPose.getY();
        double distToGoal = Math.hypot(dx, dy);

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
        telemetry.addData("Current Sensitivity", currentSensitivity);
        telemetry.addData("Sensitivity", Sensitivity);
        telemetry.addData("Precision Mode Toggle", precisionModeToggleBtn.getState());
        telemetry.addData("Precision Mode Hold", precisionModeHoldBtn.getState());

        telemetry.addLine("Debug");
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
        telemetry.addData("Flywheel Power", shooter.getFlywheelPower());
        telemetry.addData("Calculated RPM", shooter.getCalcRPM());
        telemetry.addData("Battery Voltage", "%.2f V", battery.getVoltage());
        telemetry.addLine(                                  );
        if (Auto_lastPose.currentPose != null){
        telemetry.addData("Autonomous", Auto_lastPose.currentPose);}
        telemetry.addData("X", robotPose.getX());
        telemetry.addData("Y", robotPose.getY());
        telemetry.addData("Heading", Math.toDegrees(robotPose.getHeading()));
        if (ll != null && ll.isValid()) {
            telemetry.addData("MT2_LLX", ll.getBotpose_MT2().getPosition().x);
            telemetry.addData("MT2_LLY", ll.getBotpose_MT2().getPosition().y);
            telemetry.addData("MT2_LLHeading", ll.getBotpose_MT2().getOrientation().getYaw(AngleUnit.DEGREES));
            telemetry.addData("MT_LLX", ll.getBotpose().getPosition().x);
            telemetry.addData("MT_LLY", ll.getBotpose().getPosition().y);
            telemetry.addData("MT_LLHeading", ll.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));
        } else {
            telemetry.addData("LLPose", "no valid tag");
        }
        telemetry.addData("DistanceToGoal", distToGoal);
        telemetry.update();

    }
}