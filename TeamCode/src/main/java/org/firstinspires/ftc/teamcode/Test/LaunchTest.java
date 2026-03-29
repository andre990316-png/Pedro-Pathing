package org.firstinspires.ftc.teamcode.Test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.Mechanisms.*;
import org.firstinspires.ftc.teamcode.Autonomous.Constants;

@TeleOp(name = "Launch Test", group = "Test")
public class LaunchTest extends OpMode {

    private Follower follower;
    private FlywheelController flywheel;
    private RPMVelocityConverter rpmConverter;
    private ServoAngleConverter servoConverter;
    private LaunchPhysics physics;

    private Servo hoodServo;
    private TelemetryManager telemetryM;

    private Pose goalPose = new Pose(14, 70, 0);
    private Pose startPose = new Pose(30, 125, Math.toRadians(93));

    @Override
    public void init() {
        initMotors();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        rpmConverter = new RPMVelocityConverter();
        servoConverter = new ServoAngleConverter();
        flywheel = new FlywheelController();
        physics = new LaunchPhysics();

        flywheel.init(hardwareMap, "Shooter M1");
        hoodServo = hardwareMap.get(Servo.class, "Shooter S1");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("Launch Test Ready");
        telemetry.addLine("Drive around - auto-aim adjusts continuously");
        telemetry.update();
    }

    private void initMotors() {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "Motor Front Left");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "Motor Front Right");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "Motor Back Left");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "Motor Back Right");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        autoAim();
        flywheel.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        sendTelemetry();
    }

    private void autoAim() {
        Pose robotPose = follower.getPose();

        // Step 1: Calculate ideal velocity
        LaunchPhysics.Result ideal = physics.calculate(
                robotPose.getX(), robotPose.getY(),
                goalPose.getX(), goalPose.getY()
        );

        // Step 2: Set flywheel target RPM
        double targetRPM = rpmConverter.velocityToRPM(ideal.velocity);
        flywheel.setTargetRPM(targetRPM);

        // Step 3: Use actual RPM to compute velocity
        double actualRPM = flywheel.getCurrentRPM();
        double actualVelocity = rpmConverter.rpmToVelocity(actualRPM);

        // Step 4: Calculate corrected angle
        double correctedAngle = physics.calculateAngleFromVelocity(
                robotPose.getX(), robotPose.getY(),
                goalPose.getX(), goalPose.getY(),
                actualVelocity
        );

        // Step 5: Clamp angle and convert to servo
        correctedAngle = Math.max(servoConverter.getMinAngle(), Math.min(correctedAngle, servoConverter.getMaxAngle()));
        double servoPos = servoConverter.angleToServo(correctedAngle);

        hoodServo.setPosition(servoPos);
    }

    private void sendTelemetry() {
        Pose robotPose = follower.getPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double distance = Math.hypot(dx, dy);

        telemetry.addLine("=== AUTO-AIM ===");
        telemetry.addData("Distance", "%.1f in", distance);
        telemetry.addData("Target Velocity", "%.2f m/s",
                rpmConverter.rpmToVelocity(flywheel.getTargetRPM()));
        telemetry.addData("Target Angle", "%.1f deg",
                servoConverter.servoToAngle(hoodServo.getPosition()));

        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addData("Target RPM", "%.0f", flywheel.getTargetRPM());
        telemetry.addData("Current RPM", "%.0f", flywheel.getCurrentRPM());
        telemetry.addData("Ready", flywheel.isAtTarget() ? "YES" : "NO");

        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X", "%.1f", robotPose.getX());
        telemetry.addData("Y", "%.1f", robotPose.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(robotPose.getHeading()));

        flywheel.sendTelemetry(telemetryM);
        telemetryM.addData("distance", distance);
        telemetryM.update();

        telemetry.update();
    }
}