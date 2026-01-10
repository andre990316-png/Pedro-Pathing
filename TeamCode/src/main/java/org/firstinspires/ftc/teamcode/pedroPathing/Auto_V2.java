package org.firstinspires.ftc.teamcode.pedroPathing;

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

import org.firstinspires.ftc.teamcode.mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightAim;

import java.util.ArrayList;

@Autonomous(name = "Auto_V2")
public class Auto_V2 extends OpMode {

    // Motors / hardware you already had
    private DcMotor ShooterRotateMotor;
    private LimelightAim limelight;
    private IMU imu;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Flywheel Logic
    private FlywheelLogic shooter = new FlywheelLogic();

    // Auto aim toggle
    private boolean autoAimEnabled = true;

    // ====== STEP SYSTEM ======
    public enum AutoAction {
        NONE,
        PAUSE_MS,        // uses valueMs
        INTAKE_ON,
        INTAKE_OFF,
        SHOOT_3,         // calls shooter.fireShots(3) and waits until shooter finishes
        AIM_ON,
        AIM_OFF
    }



    private ArrayList<AutoStep> STEPS = new ArrayList<>();
    private ArrayList<PathChain> CHAINS = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTEST = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPLEFT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOBOTTOMLEFT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPRIGHT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOBOTTOMRIGHT = new ArrayList<>();

    private int currentIndex = 0;
    private long pauseEndTimeMs = 0;
    private boolean waitingForShooter = false;
    private boolean timerStart = false;

    // ===== Poses you already had =====
    //start poses
    private final Pose topLeftStartPose = new Pose(20, 123, Math.toRadians(143));
    private final Pose bottomLeftStartPose = new Pose(48, 10, Math.toRadians(90));
    private final Pose bottomRightStartPose = new Pose(96, 10, Math.toRadians(90));
    private final Pose topRightStartPose = new Pose(124, 123, Math.toRadians(37));

    //close shoot poses (on big V)
    private final Pose blueShootPoseClose = new Pose(47.32299012693935, 95.86459802538788, Math.toRadians(137));
    private final Pose redShootPoseClose = new Pose(96.67700987306065, 95.86459802538788, Math.toRadians(137));

    //medium shoot poses (on big V)
    private final Pose blueShootPoseMed = new Pose(64,80,Math.toRadians(135));
    private final Pose redShootPoseMed = new Pose(80,80,Math.toRadians(45));

    //far shoot poses (on small v)
    private final Pose blueShootPoseFar = new Pose(64,20,Math.toRadians(116));
    private final Pose redShootPoseFar = new Pose(80,20,Math.toRadians(64));

    //artifact intaking poses (blue)
    private final Pose blueBallPosition1Start = new Pose(41, 35.5, Math.toRadians(180));
    private final Pose blueBallPosition1End = new Pose(10, 35.5, Math.toRadians(180));
    private final Pose blueBallPosition2Start = new Pose(41, 58, Math.toRadians(180));
    private final Pose blueBallPosition2End = new Pose(10, 58, Math.toRadians(180));
    private final Pose blueBallPosition3Start = new Pose(41, 84, Math.toRadians(180));
    private final Pose blueBallPosition3End = new Pose(17, 84, Math.toRadians(180));

    //artifact intaking poses (red)
    private final Pose redBallPosition1Start = new Pose(102, 35.5, Math.toRadians(0));
    private final Pose redBallPosition1End = new Pose(133, 35.5, Math.toRadians(0));
    private final Pose redBallPosition2Start = new Pose(102, 58, Math.toRadians(0));
    private final Pose redBallPosition2End = new Pose(133, 58, Math.toRadians(0));
    private final Pose redBallPosition3Start = new Pose(102, 84, Math.toRadians(0));
    private final Pose redBallPosition3End = new Pose(126, 84, Math.toRadians(0));

    //loading zone intaking poses (red)
    private final Pose redLoadingZoneStart = new Pose(30,12,Math.toRadians(180));
    private final Pose redLoadingZoneEnd = new Pose(10,12,Math.toRadians(180));

    //loading zone intaking poses (blue)
    private final Pose blueLoadingZoneStart = new Pose(114,12,Math.toRadians(0));
    private final Pose blueLoadingZoneEnd = new Pose(134,12,Math.toRadians(0));
    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean actionActioned=false;

    // ------------------------------------------------------------
    // Build Steps (pose + action) ONCE
    // Action runs when the segment STARTS (before followPath).
    // ------------------------------------------------------------
    private void buildSteps() {
        STEPS.clear();
        AUTOTEST.clear();
        // Start -> shoot
        AUTOTEST.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(blueShootPoseMed, AutoAction.NONE, 0));

        // At shoot pose: shoot 3, wait until shooter done (no movement)
        AUTOTEST.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        // Start moving to intake area: turn intake on at start of this segment
        AUTOTEST.add(new AutoStep(blueShootPoseMed, AutoAction.INTAKE_ON, 0));
        AUTOTEST.add(new AutoStep(blueBallPosition3Start, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(blueBallPosition3End, AutoAction.NONE, 0));

        // Start returning: turn intake off at start of this segment
        AUTOTEST.add(new AutoStep(blueBallPosition3Start, AutoAction.INTAKE_OFF, 0));
        AUTOTEST.add(new AutoStep(blueShootPoseMed, AutoAction.SHOOT_3, 0));

        // Optional: pause at shoot pose
        AUTOTEST.add(new AutoStep(null, AutoAction.PAUSE_MS, 500));



        ///top left auto

        //shoots preload, gets row 2 and shoots
        AUTOTOPLEFT.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(blueShootPoseMed, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        AUTOTOPLEFT.add(new AutoStep(blueBallPosition2End, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(blueShootPoseMed, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        //opens GATE and sucks GATE balls (great wording i know) and shoots
        //does this thrice
        for(int i=0; i<3; i++){
            AUTOTOPLEFT.add(new AutoStep(new Pose(10,62,Math.toRadians(120)), AutoAction.NONE, 0));
            AUTOTOPLEFT.add(new AutoStep(null, AutoAction.INTAKE_ON, 3000));
            AUTOTOPLEFT.add(new AutoStep(blueShootPoseMed, AutoAction.INTAKE_OFF, 0));
            AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        }

        //gets row 3 and shoots
        AUTOTOPLEFT.add(new AutoStep(blueBallPosition3Start, AutoAction.INTAKE_ON, 0));
        AUTOTOPLEFT.add(new AutoStep(blueBallPosition3End, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(blueShootPoseMed, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        //the video has more stuff but they're way faster so i think this is about as far as we're gonna get

        /// bottom left auto

        //shoot preload
        AUTOBOTTOMLEFT.add(new AutoStep(bottomLeftStartPose, AutoAction.NONE, 0));
        AUTOBOTTOMLEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        AUTOBOTTOMLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        //get loading zone balls and shoot
        //does this thrice
        for(int i=0; i<3; i++){
            AUTOBOTTOMLEFT.add(new AutoStep(blueLoadingZoneStart, AutoAction.INTAKE_ON, 0));
            AUTOBOTTOMLEFT.add(new AutoStep(blueLoadingZoneEnd, AutoAction.NONE, 0));
            AUTOBOTTOMLEFT.add(new AutoStep(blueShootPoseFar, AutoAction.INTAKE_OFF, 0));
            AUTOBOTTOMLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        }


        /// top right auto

        //flipped copy of top left
        AUTOTOPRIGHT = AutoStep.flipped(AUTOTOPLEFT);


        /// bottom right auto

        AUTOBOTTOMRIGHT = AutoStep.flipped(AUTOBOTTOMLEFT);




        STEPS.addAll(AUTOTOPLEFT);//change path here
    }

    // ------------------------------------------------------------
    // Build PathChains BETWEEN consecutive steps.
    // If either side is null -> chain is null (means "no movement").
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
    // Execute action ONCE when this index begins.
    // ------------------------------------------------------------
    private void executeAction(AutoStep step) {
        long now = System.currentTimeMillis();

        switch (step.action) {
            case NONE:
                break;

            case PAUSE_MS:
                pauseEndTimeMs = now + Math.max(0, step.valueMs);
                break;

            case INTAKE_ON:
                // You were already using shooter.getIntake().intakeReady(true)
                shooter.getIntake().intakeReady(true);
                break;

            case INTAKE_OFF:
                shooter.getIntake().intakeReady(false);
                break;

            case SHOOT_3:
                shooter.fireShots(3);
                waitingForShooter = true;
                break;

            case AIM_ON:
                autoAimEnabled = true;
                break;

            case AIM_OFF:
                autoAimEnabled = false;
                limelight.resetHistory(getRuntime());
                break;
        }
    }

    // ------------------------------------------------------------
    // Main auto sequencer
    // ------------------------------------------------------------
    private void updateAuto() {
        if (currentIndex >= CHAINS.size()) return;

        if(!timerStart) {
            if (follower.isBusy()) {
                return;
            }
            if(!actionActioned){
                AutoStep step = STEPS.get(currentIndex);
                executeAction(step);
                actionActioned=true;
            }
            if (waitingForShooter) {
                if (!shooter.isBusy()) {
                    waitingForShooter = false;
                } else {
                    return;
                }
            }
            stateTimer.reset();
            timerStart = true;
        }
        if (stateTimer.milliseconds()<pauseEndTimeMs) {
            return;
        }
        PathChain chain = CHAINS.get(currentIndex);

        if (chain != null) {
            follower.followPath(chain, true);
        }
        currentIndex++;
        stateTimer.reset();
        timerStart = false;
        waitingForShooter = false;
        actionActioned = false;

    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(topLeftStartPose);

        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        limelight = new LimelightAim(hardwareMap, imu, "Limelight");
        limelight.setPipeline("Blue");

        shooter.init(hardwareMap);

        buildSteps();
        buildChainsFromSteps();

        telemetry.addData("Init", "OK");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        currentIndex = 0;
        pauseEndTimeMs = 0;
        waitingForShooter = false;

        limelight.start(getRuntime());
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();

        // run sequencer

        updateAuto();

        // turret auto-aim
        double turretPower = 0;

        if (autoAimEnabled) {
            turretPower = limelight.update(getRuntime(), telemetry);
        } else {
            turretPower = 0;
        }
        ShooterRotateMotor.setPower(turretPower);

        telemetry.addData("Current State", STEPS.get(currentIndex).action.name());
        telemetry.addData("Step", currentIndex + " / " + CHAINS.size());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("WaitingShooter", waitingForShooter);
        telemetry.addData("PauseMsLeft", (pauseEndTimeMs > 0) ? (pauseEndTimeMs - System.currentTimeMillis()) : 0);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}