/*
package org.firstinspires.ftc.teamcode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Data.AllianceData;
import org.firstinspires.ftc.teamcode.Data.Auto_lastPose;
import org.firstinspires.ftc.teamcode.Mechanisms.LEDClass;
import org.firstinspires.ftc.teamcode.Mechanisms.LimelightAim;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;

import java.util.ArrayList;

@Autonomous(name = "Auto_V3")

public class Auto_V3 extends OpMode {

    // Motors / hardware
    private DcMotor ShooterRotateMotor;
    private LEDClass LED1 = new LEDClass();
    private LEDClass LED2 = new LEDClass();
    private LEDClass LED3 = new LEDClass();
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Flywheel Logic
    private FlywheelLogic shooter = new FlywheelLogic();

    // Auto aim
    private boolean autoAimEnabled = true;
    private Limelight3A limelight;
    private IMU imu;
    private LimelightAim autoAim = new LimelightAim();

    // ====== STEP SYSTEM ======
    public enum AutoAction {
        NONE,
        INTAKE_ON,
        INTAKE_OFF,
        SHOOT_3,
        AIM_ON,
        AIM_OFF
    }

    private ArrayList<AutoStep> STEPS = new ArrayList<>();
    private ArrayList<PathChain> CHAINS = new ArrayList<>();

    // Routine arrays
    private ArrayList<AutoStep> AUTOTEST = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPLEFT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPLEFT2 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPLEFT3 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPRIGHT2 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPRIGHT3 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOBOTTOMLEFT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPRIGHT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOBOTTOMRIGHT = new ArrayList<>();

    // Blue alliance arrays
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION1 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION2 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION3 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEGATE = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUELOADINGZONE = new ArrayList<>();
    private ArrayList<AutoStep> BLUESHOOT3NEAR = new ArrayList<>();

    // Red alliance arrays
    private ArrayList<AutoStep> INTAKEREDBALLPOSITION1 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEREDBALLPOSITION2 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEREDBALLPOSITION3 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEREDGATE = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEREDLOADINGZONE = new ArrayList<>();
    private ArrayList<AutoStep> REDSHOOT3NEAR = new ArrayList<>();

    private int currentIndex = 0;
    private long pauseEndTimeMs = 0;
    private boolean waitingForShooter = false;
    private double distToGoal = 0;

    // Alliance selection
    private boolean useRedAlliance = true; // Default to red
    private boolean allianceSelected = false;
    private final Pose blueGoalPose = new Pose(0, 144, 0);
    private final Pose redGoalPose  = new Pose(144, 144, 0);

    // ===== Poses =====
    // Start poses
    private final Pose topLeftStartPose = new Pose(20, 118.5, Math.toRadians(144));
    private final Pose bottomLeftStartPose = new Pose(48, 10, Math.toRadians(90));
    private final Pose bottomRightStartPose = new Pose(96, 10, Math.toRadians(90));
    private final Pose topRightStartPose = new Pose(124, 118.5, Math.toRadians(36));

    // Close shoot poses
    private final Pose blueShootPoseClose = new Pose(45, 96, Math.toRadians(137));
    private final Pose redShootPoseClose = new Pose(99, 96, Math.toRadians(43));

    // Medium shoot poses
    private final Pose blueShootPoseMed = new Pose(64, 80, Math.toRadians(135));
    private final Pose redShootPoseMed = new Pose(80, 80, Math.toRadians(45));

    // Far shoot poses
    private final Pose blueShootPoseFar = new Pose(64, 20, Math.toRadians(116));
    private final Pose redShootPoseFar = new Pose(80, 20, Math.toRadians(64));

    // Blue artifact intaking poses
    private final Pose blueBallPosition1Start = new Pose(50, 35.5, Math.toRadians(180));
    private final Pose blueBallPosition1End = new Pose(11, 35.5, Math.toRadians(180));
    private final Pose blueBallPosition2Start = new Pose(50, 58, Math.toRadians(180));
    private final Pose blueBallPosition2End = new Pose(11, 58, Math.toRadians(180));
    private final Pose blueBallPosition3Start = new Pose(50, 84, Math.toRadians(180));
    private final Pose blueBallPosition3End = new Pose(18, 84, Math.toRadians(180));

    // Red artifact intaking poses
    private final Pose redBallPosition1Start = new Pose(93, 35.5, Math.toRadians(0));
    private final Pose redBallPosition1End = new Pose(133, 35.5, Math.toRadians(0));
    private final Pose redBallPosition2Start = new Pose(93, 58, Math.toRadians(0));
    private final Pose redBallPosition2End = new Pose(133, 58, Math.toRadians(0));
    private final Pose redBallPosition3Start = new Pose(93, 84, Math.toRadians(0));
    private final Pose redBallPosition3End = new Pose(126, 84, Math.toRadians(0));

    // Loading zone poses
    private final Pose blueLoadingZoneStart = new Pose(34, 10, Math.toRadians(180));
    private final Pose blueLoadingZoneEnd = new Pose(11, 10, Math.toRadians(180));
    private final Pose redLoadingZoneStart = new Pose(110, 10, Math.toRadians(0));
    private final Pose redLoadingZoneEnd = new Pose(133.8, 10, Math.toRadians(0));

    // Gate intake poses
    private final Pose blueGateIntakePose = new Pose(11, 62, Math.toRadians(120));
    private final Pose redGateIntakePose = new Pose(133, 62, Math.toRadians(60));

    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean actionActioned = false;
    public int PATHNUM = 0;
    public boolean[] inputs = new boolean[]{false, false, false};
    public boolean[] lastinputs;
    public boolean[] inputpressed;
    public boolean selectedAuto = false;

    // ------------------------------------------------------------
    // Build Steps for Blue Alliance
    // ------------------------------------------------------------
    private void buildSteps_blue() {
        STEPS.clear();
        AUTOTEST.clear();
        AUTOTOPLEFT.clear();
        AUTOTOPLEFT2.clear();
        AUTOTOPLEFT3.clear();
        INTAKEBLUEBALLPOSITION1.clear();
        AUTOBOTTOMLEFT.clear();
        AUTOTOPRIGHT.clear();
        AUTOBOTTOMRIGHT.clear();
        INTAKEBLUEBALLPOSITION2.clear();
        INTAKEBLUEBALLPOSITION3.clear();
        BLUESHOOT3NEAR.clear();

        // Blue Ball Position 1
        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1End, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1Start, AutoAction.NONE, 0));

        // Blue Ball Position 2
        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2End, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2Start, AutoAction.NONE, 0));

        // Blue Ball Position 3
        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3End, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3Start, AutoAction.NONE, 0));

        // Blue Gate Intake
        INTAKEBLUEGATE.clear();
        INTAKEBLUEGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEGATE.add(new AutoStep(blueGateIntakePose, AutoAction.NONE, 4000));
        INTAKEBLUEGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.NONE, 0));

        // Blue Loading Zone
        INTAKEBLUELOADINGZONE.clear();
        INTAKEBLUELOADINGZONE.add(new AutoStep(blueLoadingZoneStart, AutoAction.INTAKE_ON, 0));
        INTAKEBLUELOADINGZONE.add(new AutoStep(blueLoadingZoneEnd, AutoAction.NONE, 0));
        INTAKEBLUELOADINGZONE.add(new AutoStep(blueLoadingZoneStart, AutoAction.NONE, 0));

        // Blue Shoot 3 Near
        BLUESHOOT3NEAR.clear();
        BLUESHOOT3NEAR.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        BLUESHOOT3NEAR.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        // Test Routine
        AUTOTEST.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(blueShootPoseMed, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTEST.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_ON, 0));
        AUTOTEST.add(new AutoStep(blueBallPosition3Start, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(blueBallPosition3End, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(blueBallPosition3Start, AutoAction.INTAKE_OFF, 0));
        AUTOTEST.add(new AutoStep(blueShootPoseClose, AutoAction.SHOOT_3, 0));
        AUTOTEST.add(new AutoStep(null, AutoAction.NONE, 500));

        // Top Left V1
        AUTOTOPLEFT.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);
        AUTOTOPLEFT.addAll(INTAKEBLUEBALLPOSITION3);
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);
        AUTOTOPLEFT.addAll(INTAKEBLUEBALLPOSITION2);
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);
        AUTOTOPLEFT.addAll(INTAKEBLUEBALLPOSITION1);
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);
        AUTOTOPLEFT.addAll(INTAKEBLUELOADINGZONE);
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);

        // Top Left V3 (with intake control)
        AUTOTOPLEFT3.add(new AutoStep(topLeftStartPose, AutoAction.INTAKE_ON, 0));
        AUTOTOPLEFT3.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT3.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT3.addAll(INTAKEBLUEBALLPOSITION3);
        AUTOTOPLEFT3.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT3.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT3.addAll(INTAKEBLUEBALLPOSITION2);
        AUTOTOPLEFT3.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT3.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT3.addAll(INTAKEBLUEBALLPOSITION1);
        AUTOTOPLEFT3.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT3.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        // Bottom Left
        AUTOBOTTOMLEFT.add(new AutoStep(bottomLeftStartPose, AutoAction.NONE, 0));
        AUTOBOTTOMLEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        AUTOBOTTOMLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        for(int i = 0; i < 3; i++) {
            AUTOBOTTOMLEFT.addAll(INTAKEBLUELOADINGZONE);
            AUTOBOTTOMLEFT.add(new AutoStep(blueShootPoseFar, AutoAction.INTAKE_OFF, 0));
            AUTOBOTTOMLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        }

        // Top Left V2
        AUTOTOPLEFT2.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT2.addAll(INTAKEBLUEBALLPOSITION3);
        AUTOTOPLEFT2.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT2.addAll(INTAKEBLUEBALLPOSITION2);
        AUTOTOPLEFT2.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT2.addAll(INTAKEBLUEBALLPOSITION1);
        AUTOTOPLEFT2.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        // Create right side routines by flipping
        AUTOTOPRIGHT = AutoStep.flipped(AUTOTOPLEFT);
        AUTOBOTTOMRIGHT = AutoStep.flipped(AUTOBOTTOMLEFT);
        AUTOTOPRIGHT2 = AutoStep.flipped(AUTOTOPLEFT2);
        AUTOTOPRIGHT3 = AutoStep.flipped(AUTOTOPLEFT3);

        // Set main routine to Top Left V3 for blue
        STEPS.addAll(AUTOTOPLEFT3);
    }

    // ------------------------------------------------------------
    // Build Steps for Red Alliance
    // ------------------------------------------------------------
    private void buildSteps_red() {
        STEPS.clear();
        AUTOTEST.clear();
        AUTOTOPRIGHT.clear();
        AUTOTOPRIGHT2.clear();
        AUTOTOPRIGHT3.clear();
        INTAKEREDBALLPOSITION1.clear();
        AUTOBOTTOMRIGHT.clear();
        INTAKEREDBALLPOSITION2.clear();
        INTAKEREDBALLPOSITION3.clear();
        REDSHOOT3NEAR.clear();

        // Red Ball Position 1
        INTAKEREDBALLPOSITION1.add(new AutoStep(redBallPosition1Start, AutoAction.INTAKE_ON, 0));
        INTAKEREDBALLPOSITION1.add(new AutoStep(redBallPosition1End, AutoAction.NONE, 0));
        INTAKEREDBALLPOSITION1.add(new AutoStep(redBallPosition1Start, AutoAction.NONE, 0));

        // Red Ball Position 2
        INTAKEREDBALLPOSITION2.add(new AutoStep(redBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEREDBALLPOSITION2.add(new AutoStep(redBallPosition2End, AutoAction.NONE, 0));
        INTAKEREDBALLPOSITION2.add(new AutoStep(redBallPosition2Start, AutoAction.NONE, 0));

        // Red Ball Position 3
        INTAKEREDBALLPOSITION3.add(new AutoStep(redBallPosition3Start, AutoAction.INTAKE_ON, 0));
        INTAKEREDBALLPOSITION3.add(new AutoStep(redBallPosition3End, AutoAction.NONE, 0));
        INTAKEREDBALLPOSITION3.add(new AutoStep(redBallPosition3Start, AutoAction.NONE, 0));

        // Red Gate Intake
        INTAKEREDGATE.clear();
        INTAKEREDGATE.add(new AutoStep(redBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEREDGATE.add(new AutoStep(redGateIntakePose, AutoAction.NONE, 4000));
        INTAKEREDGATE.add(new AutoStep(redBallPosition2Start, AutoAction.NONE, 0));

        // Red Loading Zone
        INTAKEREDLOADINGZONE.clear();
        INTAKEREDLOADINGZONE.add(new AutoStep(redLoadingZoneStart, AutoAction.INTAKE_ON, 0));
        INTAKEREDLOADINGZONE.add(new AutoStep(redLoadingZoneEnd, AutoAction.NONE, 0));
        INTAKEREDLOADINGZONE.add(new AutoStep(redLoadingZoneStart, AutoAction.NONE, 0));

        // Red Shoot 3 Near
        REDSHOOT3NEAR.clear();
        REDSHOOT3NEAR.add(new AutoStep(redShootPoseClose, AutoAction.NONE, 0));
        REDSHOOT3NEAR.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        // Red Test Routine
        AUTOTEST.add(new AutoStep(topRightStartPose, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(redShootPoseMed, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTEST.add(new AutoStep(redShootPoseClose, AutoAction.INTAKE_ON, 0));
        AUTOTEST.add(new AutoStep(redBallPosition3Start, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(redBallPosition3End, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(redBallPosition3Start, AutoAction.INTAKE_OFF, 0));
        AUTOTEST.add(new AutoStep(redShootPoseClose, AutoAction.SHOOT_3, 0));
        AUTOTEST.add(new AutoStep(null, AutoAction.NONE, 500));

        // Top Right V1
        AUTOTOPRIGHT.add(new AutoStep(topRightStartPose, AutoAction.NONE, 0));
        AUTOTOPRIGHT.addAll(REDSHOOT3NEAR);
        AUTOTOPRIGHT.addAll(INTAKEREDBALLPOSITION3);
        AUTOTOPRIGHT.addAll(REDSHOOT3NEAR);
        AUTOTOPRIGHT.addAll(INTAKEREDBALLPOSITION2);
        AUTOTOPRIGHT.addAll(REDSHOOT3NEAR);
        AUTOTOPRIGHT.addAll(INTAKEREDBALLPOSITION1);
        AUTOTOPRIGHT.addAll(REDSHOOT3NEAR);
        AUTOTOPRIGHT.addAll(INTAKEREDLOADINGZONE);
        AUTOTOPRIGHT.addAll(REDSHOOT3NEAR);

        // Top Right V3 (with intake control)
        AUTOTOPRIGHT3.add(new AutoStep(topRightStartPose, AutoAction.INTAKE_ON, 0));
        AUTOTOPRIGHT3.add(new AutoStep(redShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPRIGHT3.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPRIGHT3.addAll(INTAKEREDBALLPOSITION3);
        AUTOTOPRIGHT3.add(new AutoStep(redShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPRIGHT3.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPRIGHT3.addAll(INTAKEREDBALLPOSITION2);
        AUTOTOPRIGHT3.add(new AutoStep(redShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPRIGHT3.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPRIGHT3.addAll(INTAKEREDBALLPOSITION1);
        AUTOTOPRIGHT3.add(new AutoStep(redShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPRIGHT3.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        // Bottom Right
        AUTOBOTTOMRIGHT.add(new AutoStep(bottomRightStartPose, AutoAction.NONE, 0));
        AUTOBOTTOMRIGHT.add(new AutoStep(redShootPoseFar, AutoAction.NONE, 0));
        AUTOBOTTOMRIGHT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        for(int i = 0; i < 3; i++) {
            AUTOBOTTOMRIGHT.addAll(INTAKEREDLOADINGZONE);
            AUTOBOTTOMRIGHT.add(new AutoStep(redShootPoseFar, AutoAction.INTAKE_OFF, 0));
            AUTOBOTTOMRIGHT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        }

        // Top Right V2
        AUTOTOPRIGHT2.add(new AutoStep(topRightStartPose, AutoAction.NONE, 0));
        AUTOTOPRIGHT2.add(new AutoStep(redShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPRIGHT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPRIGHT2.addAll(INTAKEREDBALLPOSITION3);
        AUTOTOPRIGHT2.add(new AutoStep(redShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPRIGHT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPRIGHT2.addAll(INTAKEREDBALLPOSITION2);
        AUTOTOPRIGHT2.add(new AutoStep(redShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPRIGHT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPRIGHT2.addAll(INTAKEREDBALLPOSITION1);
        AUTOTOPRIGHT2.add(new AutoStep(redShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPRIGHT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        // Create left side routines by flipping
        AUTOTOPLEFT = AutoStep.flipped(AUTOTOPRIGHT);
        AUTOBOTTOMLEFT = AutoStep.flipped(AUTOBOTTOMRIGHT);
        AUTOTOPLEFT2 = AutoStep.flipped(AUTOTOPRIGHT2);
        AUTOTOPLEFT3 = AutoStep.flipped(AUTOTOPRIGHT3);

        // Set main routine to Top Right V3 for red
        STEPS.addAll(AUTOTOPRIGHT3);
    }

    // ------------------------------------------------------------
    // Build PathChains
    // ------------------------------------------------------------
    private void buildChainsFromSteps() {
        CHAINS.clear();
        for (int i = 0; i < STEPS.size() - 1; i++) {
            AutoStep start = STEPS.get(i);
            AutoStep end = STEPS.get(i + 1);
            if (end.pose == null) {
                end.setPose(start.getPose());
                CHAINS.add(null);
                continue;
            }
            PathChain chain = follower.pathBuilder()
                    .addPath(new BezierLine(start.pose, end.pose))
                    .setLinearHeadingInterpolation(start.pose.getHeading(), end.pose.getHeading())
                    .build();
            CHAINS.add(chain);
        }
    }

    // ------------------------------------------------------------
    // Execute action
    // ------------------------------------------------------------
    private void executeAction(AutoStep step) {
        long now = System.currentTimeMillis();
        if (step.valueMs > 0) {
            pauseEndTimeMs = now + step.valueMs;
        }
        switch (step.action) {
            case NONE:
                break;
            case INTAKE_ON:
                shooter.getIntake().setIntakeOnVelocity(-1);
                shooter.getIntake().intakeReady(true);
                break;
            case INTAKE_OFF:
                shooter.getIntake().intakeReady(false);
                break;
            case SHOOT_3:
                shooter.getIntake().setIntakeOnVelocity(-0.4);
                shooter.fireShots(3);
                waitingForShooter = shooter.isBusy();
                if (!waitingForShooter) actionActioned = false;
                break;
            case AIM_ON:
                autoAimEnabled = true;
                break;
            case AIM_OFF:
                autoAimEnabled = false;
                autoAim.resetHistory(getRuntime());
                break;
        }
    }

    // ------------------------------------------------------------
    // Main auto sequencer
    // ------------------------------------------------------------
    private void updateAuto() {
        if (currentIndex >= STEPS.size()) return;
        if (follower.isBusy()) return;
        if (waitingForShooter) {
            if (!shooter.isBusy()) {
                waitingForShooter = false;
            } else {
                return;
            }
        }
        if (pauseEndTimeMs > 0) {
            long now = System.currentTimeMillis();
            if (now < pauseEndTimeMs) return;
            pauseEndTimeMs = 0;
        }
        AutoStep step = STEPS.get(currentIndex);
        if (!actionActioned) {
            executeAction(step);
            actionActioned = true;
        }
        if (waitingForShooter) return;
        if (pauseEndTimeMs > 0 && System.currentTimeMillis() < pauseEndTimeMs) return;
        PathChain chain = (currentIndex < CHAINS.size()) ? CHAINS.get(currentIndex) : null;
        if (chain != null) {
            follower.followPath(chain, true);
        }
        currentIndex++;
        actionActioned = false;
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");
        LED1.init(hardwareMap, 1);
        LED2.init(hardwareMap, 2);
        LED3.init(hardwareMap, 3);
        LED1.setGreenLED(true);
        LED2.setGreenLED(true);
        LED3.setGreenLED(true);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        shooter.init(hardwareMap);
        telemetry.addData("Status", "Initialized - Select Alliance");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (!allianceSelected) {
            // Alliance selection menu
            telemetry.addLine("=== ALLIANCE SELECTION ===");
            telemetry.addLine("Press X for RED Alliance");
            telemetry.addLine("Press B for BLUE Alliance");
            telemetry.addLine("Current: " + (useRedAlliance ? "RED" : "BLUE"));
            telemetry.addLine("");
            telemetry.addLine("Press A to confirm selection");

            if (gamepad1.x) {
                useRedAlliance = true;
                telemetry.addLine("Selected: RED Alliance");
            }
            if (gamepad1.b) {
                useRedAlliance = false;
                telemetry.addLine("Selected: BLUE Alliance");
            }

            if (gamepad1.a) {
                allianceSelected = true;
                telemetry.addLine("Selection confirmed!");
                telemetry.addLine("Starting auto for " + (useRedAlliance ? "RED" : "BLUE") + " alliance");

                // Build steps and set limelight pipeline
                if (useRedAlliance) {
                    buildSteps_red();
                    limelight.pipelineSwitch(LimelightAim.pipelineFromName("Red"));
                } else {
                    buildSteps_blue();
                    limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));
                }

                // Set starting pose
                Pose start = (!STEPS.isEmpty() && STEPS.get(0).pose != null) ? STEPS.get(0).pose :
                        (useRedAlliance ? topRightStartPose : topLeftStartPose);
                follower.setStartingPose(start);
                buildChainsFromSteps();
            }
            telemetry.update();
            return;
        }

        // Auto routine selection (after alliance is selected)
        if(selectedAuto){
            telemetry.addLine("Auto selected. Press START to begin.");
            telemetry.update();
        } else {
            String[] names = {"Top Position", "Bottom Position"};
            ArrayList<ArrayList<AutoStep>> paths = new ArrayList<>();

            if (useRedAlliance) {
                paths.add(AUTOTOPRIGHT3);     // Top Right V3 for red
                paths.add(AUTOBOTTOMRIGHT);   // Bottom Right for red
            } else {
                paths.add(AUTOTOPLEFT3);      // Top Left V3 for blue
                paths.add(AUTOBOTTOMLEFT);    // Bottom Left for blue
            }

            lastinputs = new boolean[inputs.length];
            for(int i = 0; i < inputs.length; i++) {
                lastinputs[i] = inputs[i];
            }
            inputs = new boolean[]{gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.a};
            inputpressed = new boolean[inputs.length];
            for(int i = 0; i < inputs.length; i++) {
                inputpressed[i] = inputs[i] && !lastinputs[i];
            }
            if(inputpressed[0]) {
                PATHNUM--;
                if(PATHNUM < 0) PATHNUM = names.length - 1;
            }
            if(inputpressed[1]) {
                PATHNUM++;
                if(PATHNUM > names.length - 1) PATHNUM = 0;
            }
            if(inputpressed[2]) {
                STEPS.clear();
                STEPS.addAll(paths.get(PATHNUM));
                Pose start = STEPS.get(0).pose;
                follower.setStartingPose(start);
                buildChainsFromSteps();
                selectedAuto = true;
                telemetry.addLine("Auto routine selected!");
            }

            telemetry.addLine("=== AUTO ROUTINE SELECTION ===");
            telemetry.addLine("Alliance: " + (useRedAlliance ? "RED" : "BLUE"));
            telemetry.addLine("Use D-pad Up/Down to select, A to confirm");
            for(int i = 0; i < names.length; i++) {
                telemetry.addLine(names[i] + (PATHNUM == i ? " <--" : ""));
            }
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        currentIndex = 0;
        pauseEndTimeMs = 0;
        waitingForShooter = false;
        limelight.start();
        autoAim.resetHistory(getRuntime());
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        Auto_lastPose.currentPose = follower.getPose();

        // Calculate distance to goal
        Pose robotPose = follower.getPose();
        Pose goalPose = useRedAlliance ? redGoalPose : blueGoalPose;
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        distToGoal = Math.hypot(dx, dy);
        shooter.autoAim(distToGoal);

        // Run auto sequencer
        updateAuto();

        // Turret auto-aim during shooting
        if (currentIndex < STEPS.size() && STEPS.get(currentIndex).action == AutoAction.SHOOT_3) {
            double turretPower = 0;
            limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
            if (autoAimEnabled) {
                LLResult ll = limelight.getLatestResult();
                turretPower = autoAim.update(getRuntime(), ll, telemetry);
            }
            ShooterRotateMotor.setPower(turretPower);
        } else {
            ShooterRotateMotor.setPower(0);
        }

        // Telemetry
        telemetry.addLine("=== AUTO STATUS ===");
        telemetry.addData("Alliance", useRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Current Action", (currentIndex < STEPS.size()) ? STEPS.get(currentIndex).action.name() : "DONE");
        telemetry.addData("Step", currentIndex + " / " + STEPS.size());
        telemetry.addData("Robot Busy", follower.isBusy());
        telemetry.addData("Shooter Busy", shooter.isBusy());
        telemetry.addData("Waiting Shooter", waitingForShooter);
        telemetry.addData("Robot Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Distance to Goal", "%.1f inches", distToGoal);
        telemetry.update();
    }

    // ====== AutoStep class ======
    public static class AutoStep {
        public Pose pose;
        public AutoAction action;
        public long valueMs;

        public AutoStep(Pose pose, AutoAction action, long valueMs) {
            this.pose = pose;
            this.action = action;
            this.valueMs = valueMs;
        }

        public Pose getPose() {
            return pose;
        }

        public void setPose(Pose pose) {
            this.pose = pose;
        }

        public static ArrayList<AutoStep> flipped(ArrayList<AutoStep> original) {
            ArrayList<AutoStep> flipped = new ArrayList<>();
            for (AutoStep step : original) {
                Pose flippedPose = null;
                if (step.pose != null) {
                    double newX = 144 - step.pose.getX();
                    double newY = 144 - step.pose.getY();
                    double newHeading = Math.PI - step.pose.getHeading();
                    flippedPose = new Pose(newX, newY, newHeading);
                }
                flipped.add(new AutoStep(flippedPose, step.action, step.valueMs));
            }
            return flipped;
        }
    }
}
*/