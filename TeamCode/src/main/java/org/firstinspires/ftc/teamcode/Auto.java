package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.nio.file.Paths;

@Autonomous(name = "Auto")
public class Auto extends OpMode {
    private DcMotor MotorBackLeft;
    private DcMotor MotorFrontLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackRight;
    private DcMotor IntakeMotor;
    private DcMotor ShooterM1; // right
    private DcMotor ShooterM2; // left
    private DcMotor ShooterRotateMotor;
    private Servo ShooterS1;
    private Limelight3A limelight;
    private IMU imu;
    private double Kp = 0.03;
    private double Kd = 0.0025;
    private double deadband = 1.0; // degrees
    private double maxTurretPower = 1;
    private double shooterPower = 0.0;
    private boolean shooterEnabled = false;
    private double lastTx = 0;
    private double lastAimTime = 0;
    private double lastTurretPower = 0;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    //Flywheel Logic
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;
    //Intake Logic
    //private IntakeLogic intake = new IntakeLogic();
    private boolean intakeOpened = false;

    public enum PathState {
        DriveStartToShoot,
        ShootPreload,
        GoToBlue3Start,
        Blue3ToEnd,
        Blue3BackToStart,
        ReturnToShoot
    }
    PathState pathState;
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
    private PathChain driveStartPoseShootPose;
    private PathChain driveShootPoseToBlueBallPosition3Pose;
    private PathChain driveBlueBallPosition3StartToEndIntake;
    private PathChain driveBlueBallPosition3EndToStartIntake;
    private PathChain driveBlueBallPosition3PoseToShootPose;
    public void buildPaths() {
        driveStartPoseShootPose = follower.pathBuilder()
                .addPath(new BezierLine(topLeftStartPose, blueShootPoseMed))
                .setLinearHeadingInterpolation(topLeftStartPose.getHeading(), blueShootPoseMed.getHeading())
                .build();
        driveShootPoseToBlueBallPosition3Pose = follower.pathBuilder()
                .addPath(new BezierLine(blueShootPoseMed, blueBallPosition3Start))
                .setLinearHeadingInterpolation(blueShootPoseMed.getHeading(), blueBallPosition3Start.getHeading())
                .build();
        driveBlueBallPosition3StartToEndIntake = follower.pathBuilder()
                .addPath(new BezierLine(blueBallPosition3Start, blueBallPosition3End))
                .setLinearHeadingInterpolation(blueBallPosition3Start.getHeading(), blueBallPosition3End.getHeading())
                .build();
        driveBlueBallPosition3EndToStartIntake = follower.pathBuilder()
                .addPath(new BezierLine(blueBallPosition3End, blueBallPosition3Start))
                .setLinearHeadingInterpolation(blueBallPosition3End.getHeading(), blueBallPosition3Start.getHeading())
                .build();
        driveBlueBallPosition3PoseToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(blueBallPosition3Start, blueShootPoseMed))
                .setLinearHeadingInterpolation(blueBallPosition3Start.getHeading(), blueShootPoseMed.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch (pathState) {

            case DriveStartToShoot:
                follower.followPath(driveStartPoseShootPose, true);
                setPathState(PathState.ShootPreload);
                break;

            case ShootPreload:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !shooter.isBusy()) {
                        setPathState(PathState.GoToBlue3Start);
                    }
                }
                break;

            case GoToBlue3Start:
                if (!follower.isBusy()) {
                    // If you have shooter.getIntake(), keep it. Otherwise remove these lines.
                    shooter.getIntake().intakeReady(true);

                    follower.followPath(driveShootPoseToBlueBallPosition3Pose, true);
                    setPathState(PathState.Blue3ToEnd);
                }
                break;

            case Blue3ToEnd:
                if (!follower.isBusy()) {
                    follower.followPath(driveBlueBallPosition3StartToEndIntake, true);
                    setPathState(PathState.Blue3BackToStart);
                }
                break;

            case Blue3BackToStart:
                if (!follower.isBusy()) {
                    follower.followPath(driveBlueBallPosition3EndToStartIntake, true);
                    setPathState(PathState.ReturnToShoot);
                }
                break;

            case ReturnToShoot:
                if (!follower.isBusy()) {
                    follower.followPath(driveBlueBallPosition3PoseToShootPose, true);

                    // Optional: turn intake off when leaving
                    shooter.getIntake().intakeReady(false);

                    setPathState(PathState.ShootPreload); // this will wait while returning
                }
                break;

            default:
                telemetry.addLine("No state commanded.");
                break;
        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
        intakeOpened = false;
    }
    @Override
    public void init() {
        pathState = PathState.DriveStartToShoot;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Vision + turret
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(3);
        ShooterRotateMotor = hardwareMap.get(DcMotor.class, "ShooterRotateMotor");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        shooter.init(hardwareMap);
        //intake.init(hardwareMap);
        buildPaths();
        follower.setPose(topLeftStartPose);
    }
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        limelight.start();
        lastAimTime = getRuntime();
        lastTx = 0;
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        //intake.update();
        statePathUpdate();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        //follower.setPose(new Pose(limelight.getLatestResult().getBotpose_MT2().getPosition().x, limelight.getLatestResult().getBotpose_MT2().getPosition().y, limelight.getLatestResult().getBotpose_MT2().getOrientation().getYaw(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE));
        double turretPower = 0;
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double error = -tx;

            double Aim_now = getRuntime();
            double Aim_dt = Aim_now - lastAimTime;
            //if (Aim_dt <= 0) Aim_dt = 0.02;

            //double Aim_dt = 0.04;
            double dTx = (tx - lastTx) / Aim_dt;

            if (Math.abs(tx) <= deadband) {
                turretPower = 0;
            } else {
                turretPower = Kp * error - Kd * dTx;
            }

            turretPower = Range.clip(turretPower, -maxTurretPower, maxTurretPower);

            lastTx = tx;
            lastAimTime = Aim_now;

            telemetry.addData("AutoAim", "ON");
            telemetry.addData("tx", tx);
            telemetry.addData("ta", llResult.getTa());
            telemetry.addData("dTx", dTx);
            telemetry.addData("Turret PD", turretPower);
        } else {
            // No valid tag -> stop turret for safety
            turretPower = lastTurretPower;
            telemetry.addData("AutoAim", "ON (no valid tag)");
        }
        ShooterRotateMotor.setPower(-turretPower);
        lastTurretPower = turretPower;
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("pose heading deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("imu yaw deg", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.update();
    }
}