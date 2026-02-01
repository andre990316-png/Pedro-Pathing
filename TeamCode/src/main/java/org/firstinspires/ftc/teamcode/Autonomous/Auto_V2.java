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

@Autonomous(name = "Auto_V2")
public class Auto_V2 extends OpMode {

    // Motors / hardware you already had
    private DcMotor ShooterRotateMotor;

    private LEDClass LED1 = new LEDClass();
    private LEDClass LED2 = new LEDClass();
    private LEDClass LED3 = new LEDClass();
    //private Servo RGB = null;
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

    public static int stuckon=0;

    // ====== STEP SYSTEM ======
    public enum AutoAction {
        NONE,
        INTAKE_ON,
        INTAKE_OFF,
        SHOOT_3,         // calls shooter.fireShots(3) and waits until shooter finishes
        AIM_ON,
        AIM_OFF
    }

    private ArrayList<AutoStep> STEPS = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPRIGHT3 = new ArrayList<>();
    private ArrayList<PathChain> CHAINS = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTEST = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPLEFT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPLEFT2 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPLEFT3 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPLEFT4 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPRIGHT4 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPRIGHT2 = new ArrayList<>();
    private ArrayList<AutoStep> AUTOBOTTOMLEFT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOTOPRIGHT = new ArrayList<>();
    private ArrayList<AutoStep> AUTOBOTTOMRIGHT = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION1 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION2 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION3 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEGATE = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUELOADINGZONE = new ArrayList<>();
    private ArrayList<AutoStep> BLUESHOOT3NEAR = new ArrayList<>();

    private int currentIndex = 0;
    private long pauseEndTimeMs = 0;
    private boolean waitingForShooter = false;

    private double distToGoal = 0;
    private boolean timerStart = true;
    private double lastTa = 0;
    private final Pose blueGoalPose = new Pose(0, 144, 0);
    private final Pose redGoalPose  = new Pose(144, 144, 0);

    // ===== Poses you already had =====
    //start poses
    private final Pose topLeftStartPose = new Pose(20, 118.5, Math.toRadians(144));
    private final Pose bottomLeftStartPose = new Pose(48, 10, Math.toRadians(90));
    private final Pose bottomRightStartPose = new Pose(96, 10, Math.toRadians(90));
    private final Pose topRightStartPose = new Pose(124, 118.5, Math.toRadians(36));

    //close shoot poses (on big V)
    private final Pose blueShootPoseClose = new Pose(45, 96, Math.toRadians(137));
    private final Pose redShootPoseClose = new Pose(99, 96, Math.toRadians(137));

    //medium shoot poses (on big V)
    private final Pose blueShootPoseMed = new Pose(64,80,Math.toRadians(135));
    private final Pose redShootPoseMed = new Pose(80,80,Math.toRadians(45));

    //far shoot poses (on small v)
    private final Pose blueShootPoseFar = new Pose(64,20,Math.toRadians(116));
    private final Pose redShootPoseFar = new Pose(80,20,Math.toRadians(64));

    //artifact intaking poses (blue)
    private final Pose blueBallPosition1Start = new Pose(50, 35.5, Math.toRadians(180));
    private final Pose blueBallPosition1End = new Pose(11, 35.5, Math.toRadians(180));
    private final Pose blueBallPosition2Start = new Pose(50, 58, Math.toRadians(180));
    private final Pose blueBallPosition2End = new Pose(11, 58, Math.toRadians(180));
    private final Pose blueBallPosition3Start = new Pose(50, 84, Math.toRadians(180));
    private final Pose blueBallPosition3End = new Pose(18, 84, Math.toRadians(180));

    //artifact intaking poses (red)
    private final Pose redBallPosition1Start = new Pose(93, 35.5, Math.toRadians(0));
    private final Pose redBallPosition1End = new Pose(133, 35.5, Math.toRadians(0));
    private final Pose redBallPosition2Start = new Pose(93, 58, Math.toRadians(0));
    private final Pose redBallPosition2End = new Pose(133, 58, Math.toRadians(0));
    private final Pose redBallPosition3Start = new Pose(93, 84, Math.toRadians(0));
    private final Pose redBallPosition3End = new Pose(126, 84, Math.toRadians(0));

    //loading zone intaking poses (red)
    private final Pose blueLoadingZoneStart = new Pose(34,10, Math.toRadians(180));
    private final Pose blueLoadingZoneEnd = new Pose(11, 10, Math.toRadians(180));

    //loading zone intaking poses (blue)
    private final Pose redLoadingZoneStart = new Pose(110,10, Math.toRadians(0));
    private final Pose redLoadingZoneEnd = new Pose(133.8,10, Math.toRadians(0));

    //gate and intake poses
    private final Pose blueGateIntakePose = new Pose(11,62, Math.toRadians(120));
    private final Pose redGateIntakePose = new Pose(11,62, Math.toRadians(120));


    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean actionActioned=false;

    public int PATHNUM=0;
    public boolean[] inputs=new boolean[]{false, false, false};
    public boolean[] lastinputs;
    public boolean[] inputpressed;
    public boolean selectedAuto=false;

    // ------------------------------------------------------------
    // Build Steps (pose + action) ONCE
    // Action runs when the segment STARTS (before followPath).
    // ------------------------------------------------------------

    public static String[] autonames = {"top left","top right","bottom left","bottom right"};
    ArrayList<ArrayList<AutoStep>> paths = new ArrayList<>();


    private void buildSteps() {
        paths.clear();

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

        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1End, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1Start, AutoAction.NONE, 0));

        INTAKEBLUEBALLPOSITION2.clear();

        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2End, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2Start, AutoAction.NONE, 0));

        INTAKEBLUEBALLPOSITION3.clear();

        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3End, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3Start, AutoAction.NONE, 0));

        INTAKEBLUEGATE.clear();

        INTAKEBLUEGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEGATE.add(new AutoStep(blueGateIntakePose, AutoAction.NONE, 4000));
        INTAKEBLUEGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.NONE, 0));

        INTAKEBLUELOADINGZONE.clear();

        INTAKEBLUELOADINGZONE.add(new AutoStep(blueLoadingZoneStart, AutoAction.INTAKE_ON, 0));
        INTAKEBLUELOADINGZONE.add(new AutoStep(blueLoadingZoneEnd, AutoAction.NONE, 0));
        INTAKEBLUELOADINGZONE.add(new AutoStep(blueLoadingZoneStart, AutoAction.NONE, 0));

        BLUESHOOT3NEAR.clear();

        BLUESHOOT3NEAR.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        BLUESHOOT3NEAR.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

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
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);
        AUTOTOPLEFT.addAll(INTAKEBLUEBALLPOSITION3);
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);
        AUTOTOPLEFT.addAll(INTAKEBLUEBALLPOSITION2);
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);
        AUTOTOPLEFT.addAll(INTAKEBLUEBALLPOSITION1);
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);
        AUTOTOPLEFT.addAll(INTAKEBLUELOADINGZONE);
        AUTOTOPLEFT.addAll(BLUESHOOT3NEAR);

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

        AUTOTOPLEFT4.add(new AutoStep(topLeftStartPose, AutoAction.INTAKE_ON, 0));
        AUTOTOPLEFT4.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT4.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        AUTOTOPLEFT4.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        AUTOTOPLEFT4.add(new AutoStep(blueBallPosition2End, AutoAction.NONE, 0));
        AUTOTOPLEFT4.add(new AutoStep(blueGateIntakePose, AutoAction.NONE, 3000));
        AUTOTOPLEFT4.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT4.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        AUTOTOPLEFT4.addAll(INTAKEBLUEBALLPOSITION1);
        AUTOTOPLEFT4.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT4.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        AUTOTOPLEFT4.addAll(INTAKEBLUEBALLPOSITION3);
        AUTOTOPLEFT4.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT4.add(new AutoStep(null, AutoAction.SHOOT_3, 0));


        //the video has more stuff but they're way faster so i think this is about as far as we're gonna get

        /// bottom left auto

        //shoot preload
        AUTOBOTTOMLEFT.add(new AutoStep(bottomLeftStartPose, AutoAction.NONE, 0));
        AUTOBOTTOMLEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        AUTOBOTTOMLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        //get loading zone balls and shoot
        //does this thrice
        for(int i=0; i<3; i++){
            AUTOBOTTOMLEFT.addAll(INTAKEBLUELOADINGZONE);
            AUTOBOTTOMLEFT.add(new AutoStep(blueShootPoseFar, AutoAction.INTAKE_OFF, 0));
            AUTOBOTTOMLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        }
        /// top left auto V2?

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



        /// top right auto

        //flipped copy of top left
        AUTOTOPRIGHT = AutoStep.flipped(AUTOTOPLEFT);

        /// bottom right auto

        AUTOBOTTOMRIGHT = AutoStep.flipped(AUTOBOTTOMLEFT);

        AUTOTOPRIGHT2 = AutoStep.flipped(AUTOTOPLEFT2);
        AUTOTOPRIGHT4 = AutoStep.flipped(AUTOTOPLEFT4);
        AUTOTOPRIGHT3 = AutoStep.flipped(AUTOTOPLEFT3);

        if (AllianceData.isRed()) {


            STEPS.addAll(AUTOTOPRIGHT);

            telemetry.addData("Auto Path", "RED (Right Side)");
        } else {

            STEPS.addAll(AUTOTOPLEFT);

            telemetry.addData("Auto Path", "BLUE (Left Side)");
        }

        paths.add(AUTOTOPLEFT);
        paths.add(AUTOTOPRIGHT);
        paths.add(AUTOBOTTOMLEFT);
        paths.add(AUTOBOTTOMRIGHT);



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
                shooter.getIntake().setIntakeOnVelocity(-1);
                shooter.getIntake().intakeReady(true);
                break;

            case INTAKE_OFF:
                shooter.getIntake().intakeReady(false);
                break;

            case SHOOT_3:
                shooter.getIntake().setIntakeOnVelocity(-0.4);
                shooter.fireShots(3);///TODO: fix
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
        if (currentIndex >= STEPS.size()){
            stuckon=1;
            return;
        }

        // 1) 車子還在跑路徑，就不要進行下一步
        if (follower.isBusy()){
            stuckon=2;
            return;
        }

        // 2) 正在射球就卡在這裡，直到射完
        if (waitingForShooter) {
            if (!shooter.isBusy()) {
                waitingForShooter = false;
            } else {
                stuckon=3;
                return;
            }
        }


        // 3) Pause（如果你有用 PAUSE_MS）
        if (pauseEndTimeMs > 0) {
            long now = System.currentTimeMillis();
            if (now < pauseEndTimeMs) {
                stuckon=4;
                return;
            }
            pauseEndTimeMs = 0;
        }

        AutoStep step = STEPS.get(currentIndex);

        // 4) 每個 step 的 action 只執行一次（避免射球被重複觸發）
        if (!actionActioned) {
            executeAction(step);
            stuckon=5;
            actionActioned = true;
        }

        // 5) 如果這個 action 觸發了「阻塞行為」（射球 / pause），就先不要啟動 path
        if (waitingForShooter){
            stuckon=6;
            return;
        }
        if (pauseEndTimeMs > 0 && System.currentTimeMillis() < pauseEndTimeMs){
            stuckon=7;
            return;
        }

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

        //AllianceData.selectedAlliance = AllianceData.Alliance.BLUE;

        buildSteps();
        //STEPS.addAll(AUTOTOPLEFT);

        Pose start = (!STEPS.isEmpty() && STEPS.get(0).pose != null) ? STEPS.get(0).pose : topLeftStartPose;
        follower.setStartingPose(start);   // recommended for Pedro


        buildChainsFromSteps();

        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");
        LED1.init(hardwareMap, 1);
        LED2.init(hardwareMap, 2);
        LED3.init(hardwareMap, 3);
        //RGB = hardwareMap.get(Servo.class, "RGB");
        LED1.setGreenLED(true);
        LED2.setGreenLED(true);
        LED3.setGreenLED(true);
        //RGB.setPosition(0.48);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        if (AllianceData.isRed()) {
            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Red"));
            telemetry.addData("Alliance", "RED (Pipeline set to Red)");
        } else {
            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));
            telemetry.addData("Alliance", "BLUE (Pipeline set to Blue)");
        }

        shooter.init(hardwareMap);

        telemetry.addData("Init", "OK");
        telemetry.update();
    }

    @Override
    public void init_loop(){
        if(selectedAuto){
            telemetry.addLine("auto selected.");
            telemetry.update();
        }else{

            lastinputs = new boolean[inputs.length];
            for(int i=0; i<inputs.length; i++){
                lastinputs[i]=inputs[i];
            }
            inputs = new boolean[]{gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.a};
            inputpressed = new boolean[inputs.length];
            for(int i=0; i<inputs.length; i++){
                inputpressed[i] = inputs[i]&&!lastinputs[i];
            }
            if(inputpressed[0]){
                PATHNUM--;
                if(PATHNUM<0){
                    PATHNUM=autonames.length-1;
                }
            }
            if(inputpressed[1]){
                PATHNUM++;
                if(PATHNUM>autonames.length-1){
                    PATHNUM=0;
                }
            }
            if(inputpressed[2]){

                STEPS.clear();
                STEPS.addAll(paths.get(PATHNUM));
                buildChainsFromSteps();

                currentIndex = 0;
                actionActioned = false;
                waitingForShooter = false;
                pauseEndTimeMs = 0;

                follower.setStartingPose(STEPS.get(0).pose);

                if(PATHNUM==0||PATHNUM==2){
                    AllianceData.selectedAlliance = AllianceData.Alliance.BLUE;
                    limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));
                }else{
                    AllianceData.selectedAlliance = AllianceData.Alliance.RED;
                    limelight.pipelineSwitch(LimelightAim.pipelineFromName("Red"));
                }

                selectedAuto=true;
            }


            telemetry.addLine("select auto plz (A)");
            for(int i=0; i<autonames.length; i++){
                telemetry.addLine(autonames[i]+(PATHNUM==i?" <":""));
            }
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        follower.setPose(STEPS.get(0).pose);

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

        //RGB.setPosition(shooter.isFlywheelReady()? 0.48 : 0.29);
        Auto_lastPose.currentPose = follower.getPose();

        Pose robotPose = follower.getPose();
        Pose goalPose = AllianceData.getGoalPose();  // dynamically get current alliance
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        distToGoal = Math.hypot(dx, dy);
        shooter.autoAim(distToGoal);
        // run sequencer

        updateAuto();

        // turret auto-aim
        if (!(currentIndex>=STEPS.size()) && STEPS.get(currentIndex).action == AutoAction.SHOOT_3) {
            double turretPower = 0;
            limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
            if (autoAimEnabled) {
                LLResult ll = limelight.getLatestResult();
                turretPower = autoAim.update(getRuntime(), ll, telemetry);
            } else {
                turretPower = 0;
            }
            ShooterRotateMotor.setPower(turretPower);
        } else {
            ShooterRotateMotor.setPower(0);
        }

        telemetry.addData("Current State", (currentIndex < STEPS.size()) ? STEPS.get(currentIndex).action.name() : "DONE");
        telemetry.addData("Step", currentIndex + " / " + STEPS.size());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Shooter Busy", shooter.isBusy());
        telemetry.addData("WaitingShooter", waitingForShooter);
        telemetry.addData("PauseMsLeft", (pauseEndTimeMs > 0) ? (pauseEndTimeMs - System.currentTimeMillis()) : 0);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("stuckon", stuckon);
        telemetry.update();
    }
}