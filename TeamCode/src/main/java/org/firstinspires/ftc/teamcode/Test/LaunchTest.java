package org.firstinspires.ftc.teamcode.Test;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.Mechanisms.*;
import org.firstinspires.ftc.teamcode.Autonomous.Constants;
@TeleOp(name = "Launch Test", group = "Test")
public class LaunchTest extends OpMode {

    // Pedro Pathing
    private Follower follower;

    // Mechanisms
    private FlywheelController flywheel;
    private RPMVelocityConverter rpmConverter;
    private ServoAngleConverter servoConverter;
    private LaunchPhysics physics;

    // Hardware
    private Servo hoodServo;

    // Telemetry
    private TelemetryManager telemetryM;

    // Goal position (inches)
    private Pose goalPose = new Pose(14, 70, 0);  // Blue goal

    // Start position (inches)
    private Pose startPose = new Pose(30, 125, Math.toRadians(93));

    @Override
    public void init() {
        // Initialize motors
        initMotors();

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize mechanisms
        rpmConverter = new RPMVelocityConverter();
        servoConverter = new ServoAngleConverter();
        flywheel = new FlywheelController();
        physics = new LaunchPhysics();

        // Initialize hardware
        flywheel.init(hardwareMap, "Shooter M1");
        hoodServo = hardwareMap.get(Servo.class, "Shooter S1");

        // Setup telemetry
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

        // IMU
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
        // 1. Update Pedro Pathing (tracks robot position)
        follower.update();

        // 2. Auto-aim based on current position
        autoAim();

        // 3. Update flywheel control
        flywheel.update();

        // 4. Drive control
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true  // field-centric
        );

        // 5. Send telemetry
        sendTelemetry();
    }

    private void autoAim() {
        // Get current robot position
        Pose robotPose = follower.getPose();

        // Calculate required velocity and angle
        LaunchPhysics.Result result = physics.calculate(
                robotPose.getX(), robotPose.getY(),
                goalPose.getX(), goalPose.getY()
        );

        // Convert to RPM and servo position
        double targetRPM = rpmConverter.velocityToRPM(result.velocity);
        double servoPos = servoConverter.angleToServo(result.angle);

        // Apply to hardware
        flywheel.setTargetRPM(targetRPM);
        hoodServo.setPosition(servoPos);
    }

    private void sendTelemetry() {
        Pose robotPose = follower.getPose();

        // Distance to goal
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

        // Send to Panels
        flywheel.sendTelemetry(telemetryM);
        telemetryM.addData("distance", distance);
        telemetryM.update();

        telemetry.update();
    }
}