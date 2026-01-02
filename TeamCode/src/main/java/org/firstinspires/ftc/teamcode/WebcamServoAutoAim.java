package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="WebcamServoAutoAim")
public class WebcamServoAutoAim extends OpMode {

    private Servo webcamServo;
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // --- Linear mapping: 0..300 deg  <->  0.0..1.0 position ---
    private static final double SERVO_TOTAL_DEG = 300.0;
    private static final double CENTER          = 0.50;    // 150° (middle of travel)
    private static final int    TARGET_ID       = 22;      // change to your tag ID
    private static final double KP_SERVO_PER_DEG = 0.00010; // try 0.010 first; smaller = gentler
    private static final double DEAD_BAND_DEG    = 1.0;   // ignore tiny noise
    private static final int    SIGN             = -1;    // keep as-is; we'll flip if needed


    // live state
    private double position = CENTER;

    @Override
    public void init() {
        webcamServo = hardwareMap.get(Servo.class, "WebcamServo");

        // Try FORWARD first. If tag-right still pans left after testing, flip SIGN to -1.
        webcamServo.setDirection(Servo.Direction.FORWARD);

        aprilTagWebcam.init(hardwareMap, telemetry);
        webcamServo.setPosition(position);

        telemetry.addLine("WebcamServoAutoAim_Linear300 ready");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_ID);

        if (tag != null) {
            double bearing = tag.ftcPose.bearing; // + right, - left
            double absErr  = Math.abs(bearing);

            if (absErr < DEAD_BAND_DEG) {
                // hold position to kill jitter near center
                webcamServo.setPosition(position);
                telemetry.addLine("Centered (deadband)");
            } else {
                // incremental P control: move a fraction of the error each loop
                double delta = SIGN * KP_SERVO_PER_DEG * bearing; // servo units per loop
                position = Math.max(0.0, Math.min(1.0, position + delta));
                webcamServo.setPosition(position);

                telemetry.addLine("Tag detected");
                telemetry.addData("Bearing (deg)", "%.2f", bearing);
                telemetry.addData("Delta", "%.4f", delta);
            }
            telemetry.addData("Servo pos", "%.3f", position);
        } else {
            telemetry.addLine("No tag visible");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}