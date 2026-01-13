package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AprilTagLimelightTest")
public class AprilTagLimelightTest extends OpMode {
    private DcMotor ShooterRotateMotor;
    private Servo ShooterS1;
    private Limelight3A limelight;
    private IMU imu;
    private double Kp = 0.03;
    private double Kd = 0.0025;
    private double deadband = 1.0; // degrees
    private double maxTurretPower = 0.6;
    private double lastTx = 0;
    private double lastAimTime = 0;
    private double lastTurretPower = 0;
    @Override
    public void init() {
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
    }
    @Override
    public void start() {
        limelight.start();
        lastAimTime = getRuntime();
        lastTx = 0;
    }
    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        double turretPower = 0;
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double error = -tx;

            double Aim_now = getRuntime();
            double Aim_dt = Aim_now - lastAimTime;
            if (Aim_dt <= 0) Aim_dt = 0.02;
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
        telemetry.update();
    }
}