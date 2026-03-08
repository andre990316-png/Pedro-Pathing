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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Data.AllianceData;
import org.firstinspires.ftc.teamcode.Data.Auto_lastPose;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.LEDClass;
import org.firstinspires.ftc.teamcode.Mechanisms.LimelightAim;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.FTCCoordinates;

import java.util.ArrayList;

@Autonomous(name = "Auto_V2")
public class Auto_V2 extends OpMode {

    // Motors / hardware you already had
    private DcMotor ShooterRotateMotor;
    private Servo ShooterS2;

    private LEDClass LED1 = new LEDClass();
    private LEDClass LED2 = new LEDClass();
    private LEDClass LED3 = new LEDClass();
    private Servo RGB = null;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Pose goalPose;

    private double gateCloseAngle = 1;
    private double gateOpenAngle  = 0.7;
    // Flywheel Logic
    private FlywheelLogic shooter = new FlywheelLogic();

    // Intake Logic
    private IntakeLogic intake = new IntakeLogic();

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
    private ArrayList<PathChain> CHAINS = new ArrayList<>();
    private ArrayList<AutoStep> PATH1LEFT = new ArrayList<>();
    private ArrayList<AutoStep> PATH2LEFT = new ArrayList<>();
    private ArrayList<AutoStep> PATH3LEFT = new ArrayList<>();
    private ArrayList<AutoStep> PATH4LEFT = new ArrayList<>();
    private ArrayList<AutoStep> PATH5LEFT = new ArrayList<>();
    private ArrayList<AutoStep> PATH6LEFT = new ArrayList<>();
    private ArrayList<AutoStep> PATH7LEFT = new ArrayList<>();
    private ArrayList<AutoStep> PATH8LEFT = new ArrayList<>();
    private ArrayList<AutoStep> PATHNKSHRIMPLEFT = new ArrayList<>();
    private ArrayList<AutoStep> PATH1RIGHT = new ArrayList<>();
    private ArrayList<AutoStep> PATH2RIGHT = new ArrayList<>();
    private ArrayList<AutoStep> PATH3RIGHT = new ArrayList<>();
    private ArrayList<AutoStep> PATH4RIGHT = new ArrayList<>();
    private ArrayList<AutoStep> PATH5RIGHT = new ArrayList<>();
    private ArrayList<AutoStep> PATH6RIGHT = new ArrayList<>();
    private ArrayList<AutoStep> PATH7RIGHT = new ArrayList<>();
    private ArrayList<AutoStep> PATH8RIGHT = new ArrayList<>();
    private ArrayList<AutoStep> PATHNKSHRIMPRIGHT = new ArrayList<>();
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
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION2ANDOPENGATE = new ArrayList<>();
    private ArrayList<AutoStep> BLUEBALLPOSITION2ANDOPENGATE = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEBALLPOSITION3 = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUEGATE = new ArrayList<>();
    private ArrayList<AutoStep> INTAKEBLUELOADINGZONE = new ArrayList<>();
    private ArrayList<AutoStep> BLUESHOOT3NEAR = new ArrayList<>();

    private int currentIndex = 0;
    private long pauseEndTimeMs = 0;
    private boolean waitingForShooter = false;

    private double distToGoal = 0;

    // ===== Poses you already had =====
    //start poses
    private final Pose topLeftStartPose = new Pose(30, 125, Math.toRadians(93));
    private final Pose bottomLeftStartPose = new Pose(55, 0, Math.toRadians(90));

    //end poses
    private final Pose topLeftEndPose = new Pose(30, 62, Math.toRadians(180));
    private final Pose bottonLeftEndPose = new Pose(41, 12, Math.toRadians(180));

    //close shoot poses (on big V)
    private final Pose blueShootPoseClose = new Pose(50, 84, Math.toRadians(180));
    private final Pose redShootPoseClose = new Pose(94, 84, Math.toRadians(137));

    //far shoot poses (on small v)
    private final Pose blueShootPoseFar = new Pose(58,12,Math.toRadians(180));
    private final Pose redShootPoseFar = new Pose(80,20,Math.toRadians(64));

    //artifact intaking poses (blue)
    private final Pose blueBallPosition1Start = new Pose(50, 35.5, Math.toRadians(180));
    private final Pose blueBallPosition1End = new Pose(13, 35.5, Math.toRadians(180));
    private final Pose blueBallPosition2Start = new Pose(50, 62, Math.toRadians(180));
    private final Pose blueBallPosition2End = new Pose(13, 62, Math.toRadians(180));
    private final Pose blueBallPosition2EndAndGate = new Pose(11, 58, Math.toRadians(180));
    private final Pose blueBallPosition3Start = new Pose(50, 84, Math.toRadians(180));
    private final Pose blueBallPosition3End = new Pose(20, 84, Math.toRadians(180));

    //artifact intaking poses (red)
    private final Pose redBallPosition1Start = new Pose(93, 35.5, Math.toRadians(0));
    private final Pose redBallPosition1End = new Pose(133, 35.5, Math.toRadians(0));
    private final Pose redBallPosition2Start = new Pose(93, 58, Math.toRadians(0));
    private final Pose redBallPosition2End = new Pose(133, 58, Math.toRadians(0));
    private final Pose redBallPosition3Start = new Pose(93, 84, Math.toRadians(0));
    private final Pose redBallPosition3End = new Pose(126, 84, Math.toRadians(0));

    //loading zone intaking poses (red)
    private final Pose blueLoadingZoneStart = new Pose(50,11, Math.toRadians(180));
    private final Pose blueLoadingZoneEnd = new Pose(15, 11, Math.toRadians(180));

    //loading zone intaking poses (blue)
    private final Pose redLoadingZoneStart = new Pose(110,10, Math.toRadians(0));
    private final Pose redLoadingZoneEnd = new Pose(133.8,10, Math.toRadians(0));

    //gate and intake poses
    public static final Pose blueGateIntakePose = new Pose(11,54, Math.toRadians(140));
    public static final Pose redGateIntakePose = new Pose(133,54, Math.toRadians(40));

    private final Pose blueGatePose = new Pose(20, 66, Math.toRadians(180));

    //private final Pose blueGateReadyPose = new Pose(30, 63, Math.toRadians(180));

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

    public static int ALLIANCENUM = 0;
    public static int uitab=0;

    public static String[] alliancenames = {"BLUE","RED"};

    public static String[] autonames = {"topleftALL_(P1B)","topleftALL+GATE_(P2B)","bottomleft(P3B)","bottomleft+R3(P4B)","bottomleft+EXTRA(P5B)","bottomleft+R3+EXTRA(P6B)", "topleft2(P7B)", "topleft2+GATE(P8B)", "NKSHRIMPLEFT",
            "toprightALL(P1R)","toprightALL+GATE(P2R)","bottomright(P3R)","bottomright+R3(P4R)","bottomright+EXTRA(P5R)","bottomright+R3+EXTRA(P6R)", "topright2(P7R)", "topright2+GATE(P8R)", "NKSHRIMPRIGHT"};
    ArrayList<ArrayList<AutoStep>> paths = new ArrayList<>();


    private void buildSteps() {
        paths.clear();

        STEPS.clear();

        INTAKEBLUEBALLPOSITION1.clear();

        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1End, AutoAction.INTAKE_OFF, 0));
        INTAKEBLUEBALLPOSITION1.add(new AutoStep(blueBallPosition1Start, AutoAction.NONE, 0));

        INTAKEBLUEBALLPOSITION2.clear();

        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2End, AutoAction.INTAKE_OFF, 0));
        INTAKEBLUEBALLPOSITION2.add(new AutoStep(blueBallPosition2Start, AutoAction.NONE, 0));

        INTAKEBLUEBALLPOSITION3.clear();

        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3End, AutoAction.INTAKE_OFF, 0));
        INTAKEBLUEBALLPOSITION3.add(new AutoStep(blueBallPosition3Start, AutoAction.NONE, 0));

        INTAKEBLUEGATE.clear();

        INTAKEBLUEGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEGATE.add(new AutoStep(blueGateIntakePose, AutoAction.NONE, 4000));
        INTAKEBLUEGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.NONE, 0));

        INTAKEBLUELOADINGZONE.clear();

        INTAKEBLUELOADINGZONE.add(new AutoStep(blueLoadingZoneStart, AutoAction.INTAKE_ON, 0));
        INTAKEBLUELOADINGZONE.add(new AutoStep(blueLoadingZoneEnd, AutoAction.NONE, 0));
        INTAKEBLUELOADINGZONE.add(new AutoStep(blueLoadingZoneStart, AutoAction.NONE, 0));

        INTAKEBLUEBALLPOSITION2ANDOPENGATE.clear();

        INTAKEBLUEBALLPOSITION2ANDOPENGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        INTAKEBLUEBALLPOSITION2ANDOPENGATE.add(new AutoStep(blueGateIntakePose, AutoAction.NONE, 0));
        INTAKEBLUEBALLPOSITION2ANDOPENGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.NONE, 0));

        BLUESHOOT3NEAR.clear();

        BLUESHOOT3NEAR.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        BLUESHOOT3NEAR.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        BLUEBALLPOSITION2ANDOPENGATE.clear();

        BLUEBALLPOSITION2ANDOPENGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        //BLUEBALLPOSITION2ANDOPENGATE.add(new AutoStep(blueBallPosition2End, AutoAction.NONE, 0));
        //BLUEBALLPOSITION2ANDOPENGATE.add(new AutoStep(blueGateReadyPose, AutoAction.NONE, 0));
        BLUEBALLPOSITION2ANDOPENGATE.add(new AutoStep(blueGatePose, AutoAction.NONE, 1000));
        BLUEBALLPOSITION2ANDOPENGATE.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));

        //PATH1
        PATH1LEFT.clear();

        PATH1LEFT.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        PATH1LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 800));
        PATH1LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH1LEFT.addAll(INTAKEBLUEBALLPOSITION3);
        PATH1LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH1LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH1LEFT.addAll(INTAKEBLUEBALLPOSITION2);
        PATH1LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH1LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH1LEFT.addAll(INTAKEBLUEBALLPOSITION1);
        PATH1LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH1LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH1LEFT.add(new AutoStep(topLeftEndPose, AutoAction.NONE, 0));




        //PATH2
        PATH2LEFT.clear();

        PATH2LEFT.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        PATH2LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 800));
        PATH2LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH2LEFT.addAll(BLUEBALLPOSITION2ANDOPENGATE);
        PATH2LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH2LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH2LEFT.addAll(INTAKEBLUEBALLPOSITION1);
        PATH2LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH2LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH2LEFT.addAll(INTAKEBLUEBALLPOSITION3);
        PATH2LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH2LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH2LEFT.add(new AutoStep(topLeftEndPose, AutoAction.NONE, 0));

        //PATH3
        PATH3LEFT.clear();

        PATH3LEFT.add(new AutoStep(bottomLeftStartPose, AutoAction.NONE, 0));
        PATH3LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 800));
        PATH3LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH3LEFT.addAll(INTAKEBLUELOADINGZONE);
        PATH3LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        PATH3LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH3LEFT.add(new AutoStep(bottonLeftEndPose, AutoAction.NONE, 0));


        //PATH4
        PATH4LEFT.clear();

        PATH4LEFT.add(new AutoStep(bottomLeftStartPose, AutoAction.NONE, 0));
        PATH4LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 800));
        PATH4LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH4LEFT.addAll(INTAKEBLUELOADINGZONE);
        PATH4LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        PATH4LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH4LEFT.addAll(INTAKEBLUEBALLPOSITION1);
        PATH4LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        PATH4LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH4LEFT.add(new AutoStep(bottonLeftEndPose, AutoAction.NONE, 0));




        //PATH5
        PATH5LEFT.clear();

        PATH5LEFT.add(new AutoStep(bottomLeftStartPose, AutoAction.NONE, 0));
        PATH5LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 800));
        PATH5LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH5LEFT.addAll(INTAKEBLUELOADINGZONE);
        PATH5LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        PATH5LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH5LEFT.addAll(INTAKEBLUELOADINGZONE);
        PATH5LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        PATH5LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH5LEFT.add(new AutoStep(bottonLeftEndPose, AutoAction.NONE, 0));




        //PATH6
        PATH6LEFT.clear();

        PATH6LEFT.add(new AutoStep(bottomLeftStartPose, AutoAction.NONE, 0));
        PATH6LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 800));
        PATH6LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH6LEFT.addAll(INTAKEBLUELOADINGZONE);
        PATH6LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        PATH6LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH6LEFT.addAll(INTAKEBLUEBALLPOSITION1);
        PATH6LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        PATH6LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH6LEFT.addAll(INTAKEBLUELOADINGZONE);
        PATH6LEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        PATH6LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH6LEFT.add(new AutoStep(bottonLeftEndPose, AutoAction.NONE, 0));





        //PATH7
        PATH7LEFT.clear();

        PATH7LEFT.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        PATH7LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 800));
        PATH7LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH7LEFT.addAll(INTAKEBLUEBALLPOSITION2);
        PATH7LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH7LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH7LEFT.addAll(INTAKEBLUEBALLPOSITION3);
        PATH7LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH7LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH7LEFT.add(new AutoStep(topLeftEndPose, AutoAction.NONE, 0));





        //PATH8
        PATH8LEFT.clear();

        PATH8LEFT.add(new AutoStep(topLeftStartPose, AutoAction.NONE, 0));
        PATH8LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 800));
        PATH8LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH8LEFT.addAll(BLUEBALLPOSITION2ANDOPENGATE);
        PATH8LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH8LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH8LEFT.addAll(INTAKEBLUEBALLPOSITION3);
        PATH8LEFT.add(new AutoStep(blueShootPoseClose, AutoAction.NONE, 0));
        PATH8LEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATH8LEFT.add(new AutoStep(topLeftEndPose, AutoAction.NONE, 0));


        PATHNKSHRIMPLEFT.clear();

        PATHNKSHRIMPLEFT.add(new AutoStep(bottomLeftStartPose, AutoAction.NONE, 0));
        PATHNKSHRIMPLEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 800));
        PATHNKSHRIMPLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        PATHNKSHRIMPLEFT.addAll(INTAKEBLUELOADINGZONE);
        PATHNKSHRIMPLEFT.add(new AutoStep(bottonLeftEndPose, AutoAction.NONE, 0));

        /*
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

//        AUTOTOPLEFT4.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
//        AUTOTOPLEFT4.add(new AutoStep(blueBallPosition2End, AutoAction.NONE, 0));
//        AUTOTOPLEFT4.add(new AutoStep(blueGateIntakePose, AutoAction.NONE, 3000));
        AUTOTOPLEFT4.addAll(INTAKEBLUEBALLPOSITION2);
        AUTOTOPLEFT4.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT4.add(new AutoStep(null, AutoAction.SHOOT_3, 0));


        AUTOTOPLEFT4.add(new AutoStep(blueBallPosition2Start, AutoAction.INTAKE_ON, 0));
        AUTOTOPLEFT4.add(new AutoStep(blueGateIntakePose, AutoAction.NONE, 1500));
        AUTOTOPLEFT4.add(new AutoStep(blueBallPosition2Start, AutoAction.NONE, 0));
        AUTOTOPLEFT4.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT4.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        AUTOTOPLEFT4.addAll(INTAKEBLUEBALLPOSITION3);
        AUTOTOPLEFT4.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT4.add(new AutoStep(null, AutoAction.SHOOT_3, 0));

        AUTOTOPLEFT4.addAll(INTAKEBLUEBALLPOSITION1);
        AUTOTOPLEFT4.add(new AutoStep(blueShootPoseClose, AutoAction.INTAKE_OFF, 0));
        AUTOTOPLEFT4.add(new AutoStep(null, AutoAction.SHOOT_3, 0));


        //the video has more stuff but they're way faster so i think this is about as far as we're gonna get

        /// bottom left auto

        //shoot preload
        AUTOBOTTOMLEFT.add(new AutoStep(bottomLeftStartPose, AutoAction.NONE, 0));
        AUTOBOTTOMLEFT.add(new AutoStep(blueShootPoseFar, AutoAction.NONE, 0));
        AUTOBOTTOMLEFT.add(new AutoStep(null, AutoAction.SHOOT_3, 0));
        AUTOBOTTOMLEFT.addAll(INTAKEBLUEBALLPOSITION1);
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

        */

        PATH1RIGHT = AutoStep.flipped(PATH1LEFT);
        PATH2RIGHT = AutoStep.flipped(PATH2LEFT);
        PATH3RIGHT = AutoStep.flipped(PATH3LEFT);
        PATH4RIGHT = AutoStep.flipped(PATH4LEFT);
        PATH5RIGHT = AutoStep.flipped(PATH5LEFT);
        PATH6RIGHT = AutoStep.flipped(PATH6LEFT);
        PATH7RIGHT = AutoStep.flipped(PATH7LEFT);
        PATH8RIGHT = AutoStep.flipped(PATH8LEFT);
        PATHNKSHRIMPRIGHT = AutoStep.flipped(PATHNKSHRIMPLEFT);

        /*if (AllianceData.isRed()) {
            STEPS.addAll(AUTOTOPRIGHT2);

            telemetry.addData("Auto Path", "RED (Right Side)");
            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Red"));
        } else {

            STEPS.addAll(AUTOTOPLEFT2);

            telemetry.addData("Auto Path", "BLUE (Left Side)");
            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));
        }*/

//        paths.add(AUTOTOPLEFT4);
//        paths.add(AUTOTOPRIGHT4);
//        paths.add(AUTOBOTTOMLEFT);
//        paths.add(AUTOBOTTOMRIGHT);

        paths.add(PATH1LEFT);
        paths.add(PATH2LEFT);
        paths.add(PATH3LEFT);
        paths.add(PATH4LEFT);
        paths.add(PATH5LEFT);
        paths.add(PATH6LEFT);
        paths.add(PATH7LEFT);
        paths.add(PATH8LEFT);
        paths.add(PATHNKSHRIMPLEFT);

        paths.add(PATH1RIGHT);
        paths.add(PATH2RIGHT);
        paths.add(PATH3RIGHT);
        paths.add(PATH4RIGHT);
        paths.add(PATH5RIGHT);
        paths.add(PATH6RIGHT);
        paths.add(PATH7RIGHT);
        paths.add(PATH8RIGHT);
        paths.add(PATHNKSHRIMPRIGHT);



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
                ShooterS2.setPosition(gateCloseAngle);
                break;

            case INTAKE_ON:
                // You were already using shooter.getIntake().intakeReady(true)
                intake.setTargetRPM(-1);
                intake.intakeReady(true);
                break;

            case INTAKE_OFF:
                intake.intakeReady(false);
                break;

            case SHOOT_3:
                autoAimEnabled = true;
                shooter.fireShots(3);
                //intake.setTargetRPM(-0.37);
                //intake.intakeReady(true);
                //ShooterS2.setPosition(gateOpenAngle);
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
        stuckon=0;
    }

    @Override
    public void init() {
        buildSteps();
        ShooterS2 = hardwareMap.get(Servo.class, "Shooter S2");
        ShooterS2.setPosition(gateCloseAngle);
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        intake.init(hardwareMap);

//        buildSteps();
//
//        STEPS.clear();
//        STEPS.addAll(paths.get(PATHNUM));
//        buildChainsFromSteps();
//
//        currentIndex = 0;
//        actionActioned = false;
//        waitingForShooter = false;
//        pauseEndTimeMs = 0;
//
//        follower.setStartingPose(STEPS.get(0).pose);
//
//        if(PATHNUM<8){
//            AllianceData.selectedAlliance = AllianceData.Alliance.BLUE;
//            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));
//        }else{
//            AllianceData.selectedAlliance = AllianceData.Alliance.RED;
//            limelight.pipelineSwitch(LimelightAim.pipelineFromName("Red"));
//        }



        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");
        LED1.init(hardwareMap, 1);
        LED2.init(hardwareMap, 2);
        LED3.init(hardwareMap, 3);
        RGB = hardwareMap.get(Servo.class, "RGB");
        LED1.setGreenLED(true);
        LED2.setGreenLED(true);
        LED3.setGreenLED(true);
        RGB.setPosition(0.48);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        shooter.init(hardwareMap);

        telemetry.addData("Init", "OK");
        telemetry.update();
    }

    @Override
    public void init_loop(){


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
            if(uitab==0){
                ALLIANCENUM=(ALLIANCENUM+1)%2;
            }else if(uitab==1){
                PATHNUM--;
                if(AllianceData.isRed()){
                    if(PATHNUM<autonames.length/2){
                        PATHNUM=autonames.length-1;
                    }
                }else{
                    if(PATHNUM<0){
                        PATHNUM=autonames.length/2-1;
                    }
                }
            }


        }
        if(inputpressed[1]){
            if(uitab==0){
                ALLIANCENUM=(ALLIANCENUM+1)%2;
            }else if(uitab==1){
                PATHNUM++;
                if(AllianceData.isRed()){
                    if(PATHNUM>autonames.length-1){
                        PATHNUM=autonames.length/2;
                    }
                }else{
                    if(PATHNUM>autonames.length/2-1){
                        PATHNUM=0;
                    }
                }
            }


        }

        if(inputpressed[2]){
            if(uitab==0){
                uitab=1;
                if(ALLIANCENUM==0){
                    AllianceData.selectedAlliance= AllianceData.Alliance.BLUE;
                    limelight.pipelineSwitch(LimelightAim.pipelineFromName("Blue"));
                    PATHNUM=0;
                }else if(ALLIANCENUM==1){
                    AllianceData.selectedAlliance= AllianceData.Alliance.RED;
                    limelight.pipelineSwitch(LimelightAim.pipelineFromName("Red"));
                    PATHNUM=autonames.length/2;
                }
            }else if(uitab==1){
                uitab=2;
            }else if(uitab==2){
                uitab=0;
            }
        }


        if(uitab==0){//alliance select
            telemetry.addLine("select ALLIANCE plz");
            for(int i=0; i<alliancenames.length; i++){
                telemetry.addLine(alliancenames[i]+(ALLIANCENUM==i?" <":""));
            }
        }else if(uitab==1){//path select
            telemetry.addLine("select AUTO plz");
            if(AllianceData.isRed()){
                for(int i=autonames.length/2; i<autonames.length; i++){
                    telemetry.addLine(autonames[i]+(PATHNUM==i?" <":""));
                }
            }else{
                for(int i=0; i<autonames.length/2; i++){
                    telemetry.addLine(autonames[i]+(PATHNUM==i?" <":""));
                }
            }
        }else if(uitab==2){//confirm screen
            telemetry.addLine(autonames[PATHNUM]+" selected.");
            telemetry.addLine("press (A) to reset selection if you messed up");
        }


        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();

        STEPS.clear();
        STEPS.addAll(paths.get(PATHNUM));
        buildChainsFromSteps();

        currentIndex = 0;
        actionActioned = false;
        waitingForShooter = false;
        pauseEndTimeMs = 0;

        follower.setStartingPose(STEPS.get(0).pose);
        ShooterS2.setPosition(gateCloseAngle);


        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        follower.setPose(STEPS.get(0).pose);

        currentIndex = 0;
        pauseEndTimeMs = 0;
        waitingForShooter = false;

        autoAim.resetHistory(getRuntime());
    }

    @Override
    public void loop() {
        follower.update();

        shooter.update(intake);
        intake.update();

        RGB.setPosition(shooter.isFlywheelReady()? 0.48 : 0.29);
        Auto_lastPose.currentPose = follower.getPose();

        Pose robotPose = follower.getPose();
        Pose goalPose = AllianceData.getGoalPose();  // dynamically get current alliance
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        distToGoal = Math.hypot(dx, dy);
        shooter.autoAim(distToGoal);
        // run sequencer

        updateAuto();

        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
        LLResult ll = limelight.getLatestResult();

//        if(ll != null && ll.isValid()){
//            follower.setPose(getRobotPoseFromCamera()); /// questionable
//        }

        // turret auto-aim
        if (waitingForShooter) {
            double turretPower = autoAimEnabled ? autoAim.update(getRuntime(), ll, telemetry) : 0;
            ShooterRotateMotor.setPower(turretPower);
        } else {
            ShooterRotateMotor.setPower(0);
        }

///        telemetry.addData("Current State", (currentIndex < STEPS.size()) ? STEPS.get(currentIndex).action.name() : "DONE");
///        telemetry.addData("Step", currentIndex + " / " + STEPS.size());
///        telemetry.addData("Busy", follower.isBusy());
///        telemetry.addData("Shooter Busy", shooter.isBusy());
///        telemetry.addData("WaitingShooter", waitingForShooter);
///        telemetry.addData("PauseMsLeft", (pauseEndTimeMs > 0) ? (pauseEndTimeMs - System.currentTimeMillis()) : 0);
///        telemetry.addData("x", follower.getPose().getX());
///        telemetry.addData("y", follower.getPose().getY());
///        telemetry.addData("heading", follower.getPose().getHeading());
///        telemetry.addData("stuckon", stuckon);
        if (currentIndex < STEPS.size()) {
            Pose currentPose = follower.getPose();
            Pose nextPose = STEPS.get(currentIndex).pose;

            if (nextPose != null) {
                double dx2 = nextPose.getX() - currentPose.getX();
                double dy2 = nextPose.getY() - currentPose.getY();
                double distanceToNext = Math.hypot(dx2, dy2);

                ///telemetry.addData("Distance to Next Pose", "%.2f", distanceToNext);
                ///telemetry.addData("Next Pose X/Y", "%.2f / %.2f", nextPose.getX(), nextPose.getY());
            } else {
                ///telemetry.addData("Distance to Next Pose", "No target (null pose)");
            }
        } else {
            ///telemetry.addData("Distance to Next Pose", "DONE");
        }
        telemetry.addData("limelight x", getRobotPoseFromCamera().getX());
        telemetry.addData("limelight y", getRobotPoseFromCamera().getY());
        telemetry.addData("limelight h", getRobotPoseFromCamera().getHeading());

        telemetry.addData("normal x", follower.getPose().getX());
        telemetry.addData("normal y", follower.getPose().getY());
        telemetry.addData("normal h", follower.getPose().getHeading());
        telemetry.update();
    }
    private Pose getRobotPoseFromCamera() {
        ///Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        ///Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        ///Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        Pose3D pose = limelight.getLatestResult().getBotpose();
        return new Pose(pose.getPosition().x, pose.getPosition().y, follower.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}