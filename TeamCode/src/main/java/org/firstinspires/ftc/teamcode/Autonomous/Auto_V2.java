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
import org.firstinspires.ftc.teamcode.Mechanisms.LimelightAim;

import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;

import java.util.ArrayList;

@Autonomous(name = "Auto_V2")
public class Auto_V2 extends OpMode {

    // Motors / hardware you already had
    private DcMotor ShooterRotateMotor;

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Pose goalPose;

    // Flywheel Logic
    private FlywheelLogic shooter = new FlywheelLogic();

    // Auto aim toggle
    private boolean autoAimEnabled = true;
    private Limelight3A limelight;
    private IMU imu;
    private LimelightAim autoAim = new LimelightAim();

    // ====== STEP SYSTEM ======
    public enum AutoAction {
        NONE,      // uses valueMs
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
    private ArrayList<AutoStep> AUTOTOPLEFT2 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOBOTTOMLEFT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPRIGHT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOBOTTOMRIGHT = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION1 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION2 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION3 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEGATE = new ArrayList<>();

    private int currentIndex = 0;
    private long pauseEndTimeMs = 0;
    private boolean waitingForShooter = false;
    private boolean timerStart = true;
    private double lastTa = 0;
    private final Pose blueGoalPose = new Pose(0, 144, 0);
    private final Pose redGoalPose  = new Pose(144, 144, 0);

    // ===== Poses you already had =====
    //start poses
    private final Pose topLeftStartPose = new Pose(20, 123, Math.toRadians(143));
    private final Pose bottomLeftStartPose = new Pose(48, 10, Math.toRadians(90));
    private final Pose bottomRightStartPose = new Pose(96, 10, Math.toRadians(90));
    private final Pose topRightStartPose = new Pose(124, 123, Math.toRadians(37));

    //close shoot poses (on big V)
    private final Pose blueShootPoseClose = new Pose(56.05641748942172, 86.9280677009873, Math.toRadians(137));
    private final Pose redShootPoseClose = new Pose(87.9435825106, 86.9280677009873, Math.toRadians(137));

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

    //gate and intake poses
    private final Pose blueGateIntakePose = new Pose(10,62,Math.toRadians(120));
    private final Pose redGateIntakePose = new Pose(10,62,Math.toRadians(120));

    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean actionActioned=false;

    // ------------------------------------------------------------
    // Build Steps (pose + action) ONCE
    // Action runs when the segment STARTS (before followPath).
    // ------------------------------------------------------------
    private void buildSteps() {
        STEPS.clear();
        AUTOTEST.clear();
        AUTOTOPLEFT.clear();
        AUTOTOPLEFT2.clear();
        INTAKEBLUEBALLPOSITION1.clear();
        AUTOBOTTOMLEFT.clear();
        AUTOTOPRIGHT.clear();
        AUTOBOTTOMRIGHT.clear();
        INTAKEBLUEBALLPOSITION2.clear();
        INTAKEBLUEBALLPOSITION3.clear();


        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1End, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1Start, AutoAction.INTAKE_OFF, 0));

        INTAKEBLUEBALLPOSITION2.clear();

        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2End, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_OFF, 0));

        INTAKEBLUEBALLPOSITION3.clear();

        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3End, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3Start, AutoAction.INTAKE_OFF, 0));

        INTAKEBLUEGATE.clear();

        INTAKEBLUEGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEGATE.add(new AutoStep(blueGateIntakePose, AutoAction.NONE, 4000));
        INTAKEBLUEGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_OFF, 0));

        // Start -> shoot
        AUTOTEST.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(blueShootPoseMed, AutoAction.NONE, 0));

        // At shoot pose: shoot 3, wait until shooter done (no movement)
        AUTOTEST.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        // Start moving to intake area: turn intake on at start of this segment
        AUTOTEST.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_ON, 0));
        AUTOTEST.add(new AutoStep(blueBallPosition3Start, AutoAction.NONE, 0));
        AUTOTEST.add(new AutoStep(blueBallPosition3End, AutoAction.NONE, 0));

        // Start returning: turn intake off at start of this segment
        AUTOTEST.add(new AutoStep(blueBallPosition3Start, AutoAction.INTAKE_OFF, 0));
        AUTOTEST.add(new AutoStep(blueShootPoseClose, AutoAction.SHOOT_3, 0));

        // Optional: pause at shoot pose
        AUTOTEST.add(new AutoStep(null, AutoAction.NONE, 500));

        ///top left auto

        //shoots preload, gets row 2 and shoots
        AUTOTOPLEFT.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT.addAll(INTAKEBLUEBALLPOSITION3);
        AUTOTOPLEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT.addAll(INTAKEBLUEBALLPOSITION2);
        AUTOTOPLEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT.addAll(INTAKEBLUEGATE);
        AUTOTOPLEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT.addAll(INTAKEBLUEBALLPOSITION1);
        AUTOTOPLEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT.addAll(INTAKEBLUEGATE);
        AUTOTOPLEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        // top left auto V2?

        AUTOTOPLEFT2.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT2.addAll(INTAKEBLUEBALLPOSITION3);
        AUTOTOPLEFT2.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOTOPLEFT2.addAll(INTAKEBLUEBALLPOSITION2);
        AUTOTOPLEFT2.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        AUTOTOPLEFT2.addAll(INTAKEBLUEGATE);
        AUTOTOPLEFT2.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        AUTOTOPLEFT2.addAll(INTAKEBLUEGATE);
        AUTOTOPLEFT2.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        AUTOTOPLEFT2.add(new AutoStep(null, AutoAction.SHOOT_3, 0));


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
        if (step.valueMs > 0) {
            pauseEndTimeMs = now + step.valueMs;
        }
        switch (step.action) {
            case NONE:
                break;

            case INTAKE_ON:
                // You were already using shooter.getIntake().intakeReady(true)
                shooter.getIntake().intakeReady(true);
                break;

            case INTAKE_OFF:
                shooter.getIntake().intakeReady(false);
                break;

            case SHOOT_3:
                double ta = 0;
                boolean llValid = false;

                LLResult ll = limelight.getLatestResult();
                if (ll != null && ll.isValid()) {
                    llValid = true;
                    ta = ll.getTa();
                }

                FlywheelLogic.AutoShooting shot;

                if(llValid){
                    shooter.autoAim(ta);
                    lastTa = ta;
                } else {
                    shooter.autoAim(lastTa);
                }

                shooter.fireShots(3);
                waitingForShooter = shooter.isBusy();   // keep your existing blocking behavior
                if (!waitingForShooter) actionActioned = false;  // retry SHOOT_3 next loop
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

        // 1) 車子還在跑路徑，就不要進行下一步
        if (follower.isBusy()) return;

        // 2) 正在射球就卡在這裡，直到射完
        if (waitingForShooter) {
            if (shooter.isBusy()) return;   // shooter 還忙
            waitingForShooter = false;      // shooter 完成
        }

        // 3) Pause（如果你有用 PAUSE_MS）
        if (pauseEndTimeMs > 0) {
            long now = System.currentTimeMillis();
            if (now < pauseEndTimeMs) return;
            pauseEndTimeMs = 0;
        }

        AutoStep step = STEPS.get(currentIndex);

        // 4) 每個 step 的 action 只執行一次（避免射球被重複觸發）
        if (!actionActioned) {
            executeAction(step);
            actionActioned = true;
        }

        // 5) 如果這個 action 觸發了「阻塞行為」（射球 / pause），就先不要啟動 path
        if (waitingForShooter) return;
        if (pauseEndTimeMs > 0 && System.currentTimeMillis() < pauseEndTimeMs) return;

        // 6) action 都做完了，才開始跑到下一個 pose
        PathChain chain = (currentIndex < CHAINS.size()) ? CHAINS.get(currentIndex) : null;
        if (chain != null) {
            follower.followPath(chain, true);
        }

        // 7) 進下一步
        currentIndex++;
        actionActioned = false;
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildSteps();
        goalPose = blueGoalPose;
        Pose start = (!STEPS.isEmpty() && STEPS.get(0).pose != null) ? STEPS.get(0).pose : topLeftStartPose;
        follower.setStartingPose(start);   // recommended for Pedro

        buildChainsFromSteps();

        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));

        shooter.init(hardwareMap);

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

        limelight.start();
        autoAim.resetHistory(getRuntime());
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();

        Pose robotPose = follower.getPose();
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double distToGoal = Math.hypot(dx, dy);

        // run sequencer

        updateAuto();

        // turret auto-aim
        double turretPower = 0;
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
        if (autoAimEnabled) {
            LLResult ll = limelight.getLatestResult();
            turretPower = autoAim.update(getRuntime(), ll, telemetry);
        } else {
            turretPower = 0;
        }
        ShooterRotateMotor.setPower(turretPower);

        telemetry.addData("Current State", (currentIndex < STEPS.size()) ? STEPS.get(currentIndex).action.name() : "DONE");
        telemetry.addData("Step", currentIndex + " / " + STEPS.size());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Shooter Busy", shooter.isBusy());
        telemetry.addData("WaitingShooter", waitingForShooter);
        telemetry.addData("PauseMsLeft", (pauseEndTimeMs > 0) ? (pauseEndTimeMs - System.currentTimeMillis()) : 0);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("DistanceToGoal", distToGoal);
        telemetry.update();
    }
}