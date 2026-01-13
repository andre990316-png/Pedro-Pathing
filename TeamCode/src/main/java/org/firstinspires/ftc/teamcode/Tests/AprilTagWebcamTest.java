package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class AprilTagWebcamTest extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id22 = aprilTagWebcam.getTagBySpecificId(22);
        aprilTagWebcam.displayDetectionTelemetry(id22);
    }
}
