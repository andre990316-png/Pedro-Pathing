package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "DriveTrain + WebcamAutoAim", group = "TeleOp")
public class DriveTrainAndWebCamAiming extends LinearOpMode {

    // Drivetrain + intake
    private DcMotor MotorBackLeft;
    private DcMotor MotorFrontLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackRight;
    private DcMotor IntakeMotor;   // goBILDA 5203 intake motor

    // Webcam pan servo + AprilTag
    private Servo webcamServo;
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // --- Servo / aiming config for 300° servo ---
    private static final double SERVO_TOTAL_DEG = 300.0;
    private static final double CENTER          = 0.50;  // mid position ~150°
    private static final double DEAD_BAND_DEG   = 1.0;   // ignore tiny wobble
    private static final int    TARGET_ID       = 22;    // <-- set to your tag ID

    // Flip ONLY this if servo still looks away from the tag after testing
    private static final int SIGN = -1; // based on your previous behavior

    private double servoPos = CENTER;

    @Override
    public void runOpMode() {
        // ---- Map hardware ----
        MotorBackLeft   = hardwareMap.get(DcMotor.class, "MotorBackLeft");
        MotorFrontLeft  = hardwareMap.get(DcMotor.class, "MotorFrontLeft");
        MotorFrontRight = hardwareMap.get(DcMotor.class, "MotorFrontRight");
        MotorBackRight  = hardwareMap.get(DcMotor.class, "MotorBackRight");
        IntakeMotor     = hardwareMap.get(DcMotor.class, "IntakeMotor"); // <--- update config to match
        webcamServo     = hardwareMap.get(Servo.class, "WebcamServo");

        // ---- Drivetrain init ----
        MotorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        MotorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        MotorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        MotorBackRight.setDirection(DcMotor.Direction.FORWARD);

        // Intake motor: brake when stopped so it doesn’t coast
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Webcam + AprilTag init ----
        webcamServo.setDirection(Servo.Direction.FORWARD);
        servoPos = clamp(CENTER, 0.0, 1.0);
        webcamServo.setPosition(servoPos);

        aprilTagWebcam.init(hardwareMap, telemetry);

        // ---- Drivetrain state ----
        double Sensitivity = 1.0;
        int Mode = 0;
        boolean DpadUpPrev = false;
        boolean DpadDownPrev = false;
        boolean DpadLeftPrev = false;
        boolean DpadRightPrev = false;

        telemetry.addLine("DriveTrain + WebcamAutoAim READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // =========================
            // 1) AUTO-AIM WEBCAM
            // =========================
            aprilTagWebcam.update();
            AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_ID);

            if (tag != null) {
                double bearing = tag.ftcPose.bearing;  // + right, - left
                double absErr  = Math.abs(bearing);

                /*if (absErr < DEAD_BAND_DEG) {
                    // Inside deadband: hold to avoid jitter
                    webcamServo.setPosition(servoPos);
                    telemetry.addLine("Cam: centered (deadband)");
                } else {
                    // Map bearing to servo: CENTER +/- (bearing / 300)
                    double target = CENTER + SIGN * (bearing / SERVO_TOTAL_DEG);
                    servoPos = clamp(target, 0.0, 1.0);
                    webcamServo.setPosition(servoPos);

                    telemetry.addLine("Cam: tag detected");
                    telemetry.addData("Cam Bearing (deg)", "%.2f", bearing);
                }*/
                double target = CENTER + SIGN * (bearing / SERVO_TOTAL_DEG);
                servoPos = clamp(target, 0.0, 1.0);
                webcamServo.setPosition(servoPos);
                telemetry.addLine("Cam: tag detected");
                telemetry.addData("Cam Bearing (deg)", "%.2f", bearing);
                telemetry.addData("Cam ServoPos", "%.3f", servoPos);
            } else {
                telemetry.addLine("Cam: no tag visible");
            }

            // =========================
            // 2) INTAKE CONTROL (IntakeMotor)
            // =========================
            // Hold left bumper to run intake (reverse here; flip sign if spinning wrong)
            if (gamepad2.left_bumper) {
                IntakeMotor.setPower(-1.0);
            } else {
                IntakeMotor.setPower(0.0);
            }

            // =========================
            // 3) DRIVE MODE / SENSITIVITY
            // =========================
            if (gamepad2.dpad_up && !DpadUpPrev) {
                Sensitivity = Math.min(Math.max(Sensitivity + 0.1, 0.1), 1.0);
            }
            DpadUpPrev = gamepad2.dpad_up;

            if (gamepad2.dpad_down && !DpadDownPrev) {
                Sensitivity = Math.min(Math.max(Sensitivity - 0.1, 0.1), 1.0);
            }
            DpadDownPrev = gamepad2.dpad_down;

            if (gamepad2.dpad_right && !DpadRightPrev) {
                Mode = 1;
                gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }
            DpadRightPrev = gamepad1.dpad_right;

            if (gamepad2.dpad_left && !DpadLeftPrev) {
                Mode = 0;
                gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            }
            DpadLeftPrev = gamepad1.dpad_left;

            // =========================
            // 4) MECANUM DRIVE
            // =========================
            double XL = gamepad2.left_stick_x  * Sensitivity;
            double YL = -gamepad2.left_stick_y * Sensitivity; // invert Y for forward
            double XR = gamepad2.right_stick_x * Sensitivity;

            // Optional: lock out strafing in Mode 1 when mostly driving straight
            if (Mode == 1 && Math.abs(XL) < 0.2 && Math.abs(YL) > 0.2) {
                XL = 0;
            }

            double fl = YL + XL + XR;
            double fr = YL - XL - XR;
            double bl = YL - XL + XR;
            double br = YL + XL - XR;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br))));

            MotorFrontLeft.setPower(fl / max);
            MotorFrontRight.setPower(fr / max);
            MotorBackLeft.setPower(bl / max);
            MotorBackRight.setPower(br / max);

            // =========================
            // 5) TELEMETRY SUMMARY
            // =========================
            telemetry.addData("Intake Power", "%.2f", IntakeMotor.getPower());
            telemetry.addData("Drive Mode", Mode);
            telemetry.addData("Sensitivity", "%.2f", Sensitivity);
            telemetry.update();
        }

        aprilTagWebcam.stop();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}