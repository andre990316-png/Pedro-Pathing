package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.geometry.BezierPoint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2;
import org.firstinspires.ftc.teamcode.Autonomous.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Data.AllianceData;
import org.firstinspires.ftc.teamcode.Data.Auto_lastPose;
import org.firstinspires.ftc.teamcode.Mechanisms.ButtonLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.LEDClass;
import org.firstinspires.ftc.teamcode.Mechanisms.LimelightAim;
import org.firstinspires.ftc.teamcode.Tests.AutoShooting;
import org.firstinspires.ftc.teamcode.Data.FlywheelAndHoodData;
import org.firstinspires.ftc.teamcode.Mechanisms.TeleopGate;
import org.firstinspires.ftc.teamcode.Mechanisms.PatternLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.ColorSensorLogic;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    // Drive + mechanisms
    private DcMotor MotorBackLeft;
    private DcMotor MotorFrontLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackRight;
    private DcMotor ShooterM1;
    private DcMotor ShooterM2;
    private DcMotor ShooterRotateMotor;
    private Servo ShooterS1;
    private Servo ShooterS2;
    // Vision + turret
    private Limelight3A limelight;
    private IMU imu;
    private LEDClass LED1 = new LEDClass();
    private LEDClass LED2 = new LEDClass();
    private LEDClass LED3 = new LEDClass();
    private Servo RGB = null;
    private LimelightAim autoAim = new LimelightAim();
    private boolean precisionMode;
    private boolean poseSnapped = false;
    private boolean AutoPoseAvailable = false;
    private boolean AutoPoseOn = false;

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
    //Pattern
    private boolean patternLocked = false;
    private PatternLogic patternLogic = new PatternLogic();
    private int detectedTag = -1;
    private double side;

    private IntakeLogic intake = new IntakeLogic();
    private FlywheelLogic shooter = new FlywheelLogic();

    private ButtonLogic hoodUpBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);  // dpad_right
    private ButtonLogic hoodDownBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);  // dpad_left
    private ButtonLogic rpmUpBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);  // dpad_up
    private ButtonLogic rpmDownBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);  // dpad_down

    private ButtonLogic intakeUpBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);
    private ButtonLogic intakeDownBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);

    private ButtonLogic gateHoldBtn    = new ButtonLogic(ButtonLogic.Mode.HOLD, false);   // right_trigger
    private ButtonLogic shoot3Btn = new ButtonLogic(ButtonLogic.Mode.HOLD, false);   // right_trigger
    private ButtonLogic autoAimHoldBtn = new ButtonLogic(ButtonLogic.Mode.HOLD, false);   // left_trigger

    private ButtonLogic intakeHoldBtn = new ButtonLogic(ButtonLogic.Mode.HOLD, false); // gamepad1.left_bumper
    private ButtonLogic intakeReverseHoldBtn = new ButtonLogic(ButtonLogic.Mode.HOLD,false);
    private ButtonLogic autoFlywheelAndHoodToggleBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false); // gamepad2.right_bumper
    private ButtonLogic autoSort = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private ButtonLogic greenBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);  //gamepad1.a
    private ButtonLogic purpleBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false); //gamepad1.x
    private ButtonLogic patternResetBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false); //gamepad1.start
    private ButtonLogic patternConfirmBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false); //gamepad1.right_bumper
    private ButtonLogic precisionModeToggleBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);//gamepad1.right_bumper
    private ButtonLogic precisionModeHoldBtn = new ButtonLogic(ButtonLogic.Mode.HOLD, false);//gamepad1.right_trigger
    private ButtonLogic sensitivityUpBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);//gamepad1.dpad_up
    private ButtonLogic sensitivityDownBtn = new ButtonLogic(ButtonLogic.Mode.PULSE, false);//gamepad1.dpad_down
    private ButtonLogic fieldOrientedModeToggleBtn = new ButtonLogic(ButtonLogic.Mode.TOGGLE, false);
    private Follower follower;

    private Pose selectedStartPose;
    private final Pose topLeftStartPose = new Pose(30, 125, Math.toRadians(93));
    private final Pose topRightStartPose = new Pose(114, 125, Math.toRadians(87));
    private final java.util.ArrayList<PatternLogic.Color> driverPatternEntry = new java.util.ArrayList<>(3);
    private ColorSensorLogic colorSensorLogic = new ColorSensorLogic();

    private boolean autogating = false;
    private boolean autosuctiongating = false;
    private boolean parking = false;

    public static BezierPoint bluegate = new BezierPoint(14, 70);
    public static BezierPoint redgate = new BezierPoint(132.5, 70);
    public static BezierPoint redpark = new BezierPoint(29, 41.8);
    public static BezierPoint bluepark = new BezierPoint(105.4, 36.5);
    private boolean isHoldingPosition = false;

    @Override
    public void init() {
        // ===== Hardware map =====
        MotorBackLeft = hardwareMap.get(DcMotor.class, "Motor Back Left");
        MotorFrontLeft = hardwareMap.get(DcMotor.class, "Motor Front Left");
        MotorFrontRight = hardwareMap.get(DcMotor.class, "Motor Front Right");
        MotorBackRight = hardwareMap.get(DcMotor.class, "Motor Back Right");
        MotorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShooterM1 = hardwareMap.get(DcMotor.class, "Shooter M1");
        ShooterM2 = hardwareMap.get(DcMotor.class, "Shooter M2");
        ShooterS1 = hardwareMap.get(Servo.class, "Shooter S1");
        ShooterS2 = hardwareMap.get(Servo.class, "Shooter S2");
        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");
        battery = hardwareMap.voltageSensor.iterator().next();
        LED1.init(hardwareMap, 1);
        LED2.init(hardwareMap, 2);
        LED3.init(hardwareMap, 3);
        RGB = hardwareMap.get(Servo.class, "RGB");
        RGB.setPosition(0.48);
        ColorSensorLogic.init(hardwareMap);
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

        ShooterM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setTargetRPM(0);

        currentSensitivity = 1.0;
        Sensitivity = 1.0;

        intake.init(hardwareMap);

        if (Auto_lastPose.currentPose != null){
            selectedStartPose = Auto_lastPose.currentPose;
            AutoPoseAvailable = true;
        }else{
            selectedStartPose = AllianceData.isRed() ? topRightStartPose : topLeftStartPose;
            AutoPoseAvailable = false;
        }

        follower = Constants.createFollower(hardwareMap);


        telemetry.addData("Initialize", "Completed");
        telemetry.update();
    }

    @Override
    public void init_loop() {

        if (gamepad1.dpad_left) {
            AllianceData.selectedAlliance = AllianceData.Alliance.BLUE;
            selectedStartPose = topLeftStartPose;
        }
        else if (gamepad1.dpad_right) {
            AllianceData.selectedAlliance = AllianceData.Alliance.RED;
            selectedStartPose = topRightStartPose;
        }

        if (gamepad1.dpad_down && Auto_lastPose.currentPose != null) {
            selectedStartPose = Auto_lastPose.currentPose;
        }


        telemetry.addLine("=== ALLIANCE SELECT ===");
        telemetry.addData("Alliance", AllianceData.selectedAlliance);
        telemetry.addLine("D-pad LEFT = BLUE");
        telemetry.addLine("D-pad RIGHT = RED");
        telemetry.addLine(AutoPoseAvailable ? "D-pad DOWN = Auto" : "D-pad DOWN = Auto (NOT AVAILABLE)");
        telemetry.addLine("                                 ");
        telemetry.addData("Selected Start Pose", selectedStartPose);


        telemetry.update();
    }


    @Override
    public void start() {
        shooter.init(hardwareMap);
        follower.setStartingPose(selectedStartPose);
        limelight.start();
        autoAim.resetHistory(getRuntime());

        if (AllianceData.isRed()) {
            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Red"));
            side = 1;

        } else {
            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));
            side = -1;
        }
        allianceBannerTimer.reset();
        showAllianceBanner = true;
        ShooterS1.setPosition(0.84);

    }



    //====================
    //Main Loop
    //====================
    @Override
    public void loop() {
        shooter.update(intake);
        intake.update();
        follower.updatePose();
        ColorSensorLogic.update(telemetry);
        int[] colors = colorSensorLogic.returnCurrentColors();
        for (int i=0; i<3; i++) {
            if (colors[i] == 0) {
                patternLogic.addArtifact(PatternLogic.Color.U);
                LED1.setGreenLED(false);
                LED2.setGreenLED(false);
                LED3.setGreenLED(false);
                LED1.setRedLED(false);
                LED2.setRedLED(false);
                LED3.setRedLED(false);
            }
            else if (colors[i] == 1) {
                patternLogic.addArtifact(PatternLogic.Color.P);
                LED1.setGreenLED(false);
                LED2.setGreenLED(false);
                LED3.setGreenLED(false);
                LED1.setRedLED(true);
                LED2.setRedLED(true);
                LED3.setRedLED(true);
            }
            else {
                patternLogic.addArtifact(PatternLogic.Color.G);
                LED1.setGreenLED(true);
                LED2.setGreenLED(true);
                LED3.setGreenLED(true);
                LED1.setRedLED(false);
                LED2.setRedLED(false);
                LED3.setRedLED(false);
            }
        }
        LED1.setGreenLED(true);
        RGB.setPosition(shooter.isFlywheelReady()? 0.48 : 0.29);
        //teleopGate.update(shooter.getTargetRPM(), gamepad2.right_trigger > 0.3);
        LLResult ll = limelight.getLatestResult();

/*        if (!poseSnapped && ll != null && ll.isValid()) {
            // Example: botpose_MT2 gives a Pose3D-like object in FTC SDK
            double x = (ll.getBotpose().getPosition().x - 1.83) * 39.3442622951;
            double y = (ll.getBotpose().getPosition().y + 1.83) * 39.3442622951;
            double yawDeg = ll.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);

            // If Limelight is in meters and Pedro is in inches, convert:
            // x *= 39.3701; y *= 39.3701;


            Pose snapped = new Pose(x, y, Math.toRadians(yawDeg));
            follower.setPose(snapped);      // or follower.setStartingPose(snapped) depending on your Pedro version
            poseSnapped = true;
       }
        */

        //Update Buttons
        hoodUpBtn.update(gamepad2.dpad_right);
        hoodDownBtn.update(gamepad2.dpad_left);
        rpmUpBtn.update(gamepad2.dpad_up);
        rpmDownBtn.update(gamepad2.dpad_down);
        shoot3Btn.update(gamepad2.right_trigger > 0.3);
        gateHoldBtn.update(gamepad2.right_trigger > 0.03);
        autoAimHoldBtn.update(gamepad2.left_trigger > 0.3);
        autoSort.update(gamepad2.left_bumper);
        greenBtn.update(gamepad1.a);
        purpleBtn.update(gamepad1.x);
        patternResetBtn.update(gamepad1.start);
        patternConfirmBtn.update(gamepad1.right_bumper);
        intakeUpBtn.update(gamepad1.dpad_right);
        intakeDownBtn.update(gamepad1.dpad_left);
        intakeHoldBtn.update(gamepad1.left_trigger > 0.3);
        intakeReverseHoldBtn.update(gamepad1.left_bumper);
        autoFlywheelAndHoodToggleBtn.update(gamepad2.right_bumper);
        fieldOrientedModeToggleBtn.update(gamepad1.right_bumper);
        precisionModeHoldBtn.update(gamepad1.right_trigger > 0.3);
        //precisionModeToggleBtn.update(gamepad1.right_bumper);
        sensitivityDownBtn.update(gamepad1.dpad_down);
        sensitivityUpBtn.update(gamepad1.dpad_up);

        ShooterS2.setPosition(gateHoldBtn.getState()? 0.7 : 1);
        /*if (shoot3Btn.getState() && !shooter.isBusy()) {
            shooter.fireShots(3);
            shooter.getIntake().setIntakeOnVelocity(-0.6);
            shooter.getIntake().intakeReady(true);
        }else if (shoot3Btn.getState()) {
            shooter.getIntake().setIntakeOnVelocity(-0.6);
            shooter.getIntake().intakeReady(true);
        }*/

        if (gateHoldBtn.getState()) {
            intake.setTargetRPM(-0.39);
            intake.intakeReady(true);
        }

        if (!shooter.isBusy() && intakeHoldBtn.getState()) {
            intake.setTargetRPM(-1);
            intake.intakeReady(true);
        } else if (!shooter.isBusy() && intakeReverseHoldBtn.getState() && !shoot3Btn.getState() && !intakeHoldBtn.getState()) {
            intake.setTargetRPM(0.4);
            intake.intakeReady(true);
        } else if (!shooter.isBusy() && !shoot3Btn.getState() && !intakeHoldBtn.getState() && !intakeReverseHoldBtn.getState()){
            intake.intakeReady(false);
        }

        if (showAllianceBanner) {
            if (allianceBannerTimer.seconds() < .6767) {
                telemetry.addLine("=== ALLIANCE LOCKED ===");
                telemetry.addData("Alliance", AllianceData.selectedAlliance);
                telemetry.addData("Start Pose", AutoPoseOn ? "AUTO" : "TOP LEFT");
                telemetry.update();
                return; // optional — remove if you want normal telemetry underneath
            } else {
                showAllianceBanner = false;
            }
        }

//        if (precisionModeToggleBtn.getState()) {
//            precisionMode = !precisionMode;
//        }

        precisionMode = precisionModeHoldBtn.getState();

        currentSensitivity = (precisionModeToggleBtn.getState() || precisionMode) ? 0.3 : Sensitivity;

        if (sensitivityUpBtn.getState()) {
            Sensitivity = Range.clip(Sensitivity + 0.1, 0, 1);
            currentSensitivity = Sensitivity;
        }

        if (sensitivityDownBtn.getState()) {
            Sensitivity = Range.clip(Sensitivity - 0.1, 0, 1);
            currentSensitivity = Sensitivity;
        }

        // Raw joystick (FIELD intent)
        double fieldStrafe  = gamepad1.left_stick_x;
        double fieldForward = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x * 0.75;

        double strafe;
        double forward;

        if (fieldOrientedModeToggleBtn.getState()) {
            double heading = follower.getPose().getHeading();

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            strafe  = fieldStrafe * cos + fieldForward * sin;
            forward = -fieldStrafe * sin + fieldForward * cos;
            strafe *= side;
            forward *= side;
        } else {
            strafe  = fieldStrafe;
            forward = fieldForward;
        }

        strafe  *= currentSensitivity;
        forward *= currentSensitivity;
        rotation *= currentSensitivity * 0.75;

        XL = strafe;
        YL = forward;
        XR = rotation;

        TempMax1 = Math.max(Math.abs(YL + XL + XR), Math.abs((YL - XL) - XR));
        TempMax2 = Math.max(Math.abs((YL - XL) + XR), Math.abs((YL + XL) - XR));
        MaxPower = Math.max(TempMax1, TempMax2);

        autogating = gamepad1.x;
        autosuctiongating = gamepad1.y;
        parking = gamepad1.b;

        if(autogating){
            if (!isHoldingPosition){
            if(AllianceData.isRed()){
                follower.holdPoint(redgate, Math.toRadians(90));
            }else{
                follower.holdPoint(bluegate, Math.toRadians(90));
            }
                isHoldingPosition = true;
            }
            follower.update();
        }else if(autosuctiongating){
            if (!isHoldingPosition) {
                if (AllianceData.isRed()) {
                    follower.holdPoint(Auto_V2.redGateIntakePose);
                } else {
                    follower.holdPoint(Auto_V2.blueGateIntakePose);
                }
                isHoldingPosition = true;
            }
            follower.update();
        }else if (parking) {
            // 只有在 "還沒鎖定" 的時候，才發送一次指令
            if (!isHoldingPosition) {
                if (AllianceData.isRed()) {
                    follower.holdPoint(redpark, Math.toRadians(-90));
                } else {
                    follower.holdPoint(bluepark, Math.toRadians(-90));
                }
                isHoldingPosition = true;
            }
            follower.update();
        }else{
            // 如果剛剛是停車模式，現在手放開了 B 鍵 -> 解除鎖定，把控制權還給手把
            if (isHoldingPosition) {
                follower.breakFollowing();
                //follower.startTeleopDrive();
                isHoldingPosition = false;
                follower.update();

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
        }


        double yaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(yaw);

        double turretPower;
        if (autoAimHoldBtn.getState()) {
            turretPower = autoAim.update(getRuntime(), ll, telemetry);
        } else {
            double manual = gamepad2.right_stick_x;
            turretPower = (Math.abs(manual) > 0.08) ? Range.clip(manual * 0.6, -0.8, 0.8) : 0;
            autoAim.resetHistory(getRuntime());
        }
        ShooterRotateMotor.setPower(turretPower);

        Pose robotPose = follower.getPose();
        double dx = AllianceData.getGoalPose().getX() - robotPose.getX();
        double dy = AllianceData.getGoalPose().getY() - robotPose.getY();
        double distToGoal = Math.hypot(dx, dy);

/*        if(autoFlywheelAndHoodToggleBtn.justPressed()) shooter.setTargetRPM(0);
        if (autoFlywheelAndHoodToggleBtn.getState()) {
            if(llValid) {
                shot = (ta >= 0.7) ? FlywheelAndHoodData.lookupA(ta) : FlywheelAndHoodData.lookupB(ta);
                shooter.setTargetRPM(shot.rpm);
                shooter.setHoodPosition(shot.hood);
            }
        } else {
            if (rpmUpBtn.getState()) shooter.setTargetRPM(shooter.getTargetRPM() + 50);
            if (rpmDownBtn.getState()) shooter.setTargetRPM(shooter.getTargetRPM() - 50);
            if (hoodUpBtn.getState()) pos += 0.01;
            if (hoodDownBtn.getState()) pos -= 0.01;
            pos = Range.clip(pos, 0.84, 1.0);
            ShooterS1.setPosition(pos);
        };d
        */

        if(autoFlywheelAndHoodToggleBtn.justPressed()) shooter.setTargetRPM(0);
        if (autoFlywheelAndHoodToggleBtn.getState()) {
            shooter.autoAim(distToGoal);
        } else {
            if (rpmUpBtn.getState()) shooter.setTargetRPM(shooter.getTargetRPM() + 50);
            if (rpmDownBtn.getState()) shooter.setTargetRPM(shooter.getTargetRPM() - 50);
            if (hoodUpBtn.getState()) pos += 0.01;
            if (hoodDownBtn.getState()) pos -= 0.01;
            pos = Range.clip(pos, 0.84, 1.0);
            ShooterS1.setPosition(pos);
        }
       /* if(intakeUpBtn.justPressed()){
            IntakeLogic.intakeOnVelocity+=0.05;
            if(IntakeLogic.intakeOnVelocity>0){
                IntakeLogic.intakeOnVelocity=0;
            }
        }
        if(intakeDownBtn.justPressed()){
            IntakeLogic.intakeOnVelocity-=0.05;
            if(IntakeLogic.intakeOnVelocity<-1){
                IntakeLogic.intakeOnVelocity=-1;
            }
        }*/

        if (!patternLocked) {

            if (greenBtn.justPressed() && driverPatternEntry.size() < 3)
                driverPatternEntry.add(PatternLogic.Color.G);

            if (purpleBtn.justPressed() && driverPatternEntry.size() < 3)
                driverPatternEntry.add(PatternLogic.Color.P);
            /// TODO: add LED lights to confirm selection‎‎

            if (patternResetBtn.justPressed()) {
                driverPatternEntry.clear();
                patternLogic.clearAll();
                patternLocked = false;
            }

            if (patternConfirmBtn.justPressed() && driverPatternEntry.size() == 3) {
                patternLogic.setDesiredPatternFromEntry(driverPatternEntry);
                patternLocked = true;
                driverPatternEntry.clear();
            }
        }
        // Telemetry
        telemetry.addLine("In-Game");
        telemetry.addLine("                                  ");
        telemetry.addData("Driver Entry", driverPatternEntry);
        telemetry.addData("Desired Pattern (brain)", patternLogic.getPatternSnapshot());
        telemetry.addData("Pattern Locked", patternLocked);
        telemetry.addLine("                                  ");
        telemetry.addData("Target RPM ", shooter.getTargetRPM());
        telemetry.addData("RPM ", shooter.getFlywheelRpm());
        telemetry.addLine("                                  ");
        telemetry.addData("Shooter Servo ", ShooterS1.getPosition());
        telemetry.addData("Shooter Servo2 ", ShooterS2.getPosition());
        telemetry.addLine("                                  ");
        telemetry.addData("Current Sensitivity", currentSensitivity);
        telemetry.addData("Sensitivity", Sensitivity);
        telemetry.addData("Precision Mode Hold", precisionModeHoldBtn.getState());
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
        telemetry.addLine("                                  ");
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("RPM", shooter.getFlywheelRpm());
        telemetry.addData("Flywheel Error", shooter.getError());
        telemetry.addData("Flywheel Power", shooter.getFlywheelPower());
        telemetry.addData("Calculated RPM", shooter.getCalcRPM());
        telemetry.addData("Battery Voltage", "%.2f V", battery.getVoltage());
        telemetry.addLine(""                       );
        telemetry.addData("Start Pose", AutoPoseOn ? "AUTO" : "TOP LEFT");
        if (AutoPoseOn){
        telemetry.addData("Autonomous", Auto_lastPose.currentPose);}
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
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
        telemetry.addData("Intake Target RPM", intake.getTargetRPM());
        telemetry.addData("Intake RPM", intake.getCurrentRPM());
        telemetry.addData("Intake PID Output", intake.getPidPower());

        telemetry.update();

    }
}