package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Autonomous
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private CRServo limelightServo;
    private IMU imu;
    private double distance;
    private double Kp = 0.01;
    private double Kd = 0.003;

    private double lastTx = 0;
    private double lastTime = 0;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(3);
        limelightServo = hardwareMap.get(CRServo.class, "ShooterRotateServo");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    @Override
    public void start() {
        limelight.start();
        lastTime = getRuntime();
        lastTx = 0;
    }
    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid())
        {
            double tx = llResult.getTx();
            double error = -tx;

// time step
            double now = getRuntime();
            double dt = now - lastTime;
            if (dt <= 0) dt = 0.02; // safety fallback

// derivative of tx
            double dTx = (tx - lastTx) / dt;

// deadband
            double deadband = 1.0;

            double output;
            if (Math.abs(tx) <= deadband) {
                output = 0;
            } else {
                output = Kp * error - Kd * dTx;
            }

// clamp power
            output = Range.clip(output, -0.3, 0.3);
            limelightServo.setPower(output);

// update history
            lastTx = tx;
            lastTime = now;

            telemetry.addData("tx", tx);
            telemetry.addData("error", error);
            telemetry.addData("dTx", dTx);
            telemetry.addData("PD output", output);
        }
        telemetry.addData("Yaw", orientation.getYaw());
    }
    public double getDistanceFromTag(double ta){
        double scale = 30665.95;
        double distance = scale/ta;
        return distance;
    }
}
