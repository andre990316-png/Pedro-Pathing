package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Flywheel encoder "one revolution" test.
 *
 * Goal: determine how many encoder counts = 1 output-shaft revolution.
 *
 * How to use:
 * 1) Run this opmode.
 * 2) Press A to "zero" (capture start encoder value).
 * 3) Rotate the flywheel/output shaft EXACTLY 1 full turn by hand (use tape mark).
 * 4) Press B to "sample" (capture end value and show delta).
 * 5) Delta ≈ 28  -> use TICKS_PER_REV = 28
 *    Delta ≈ 112 -> use TICKS_PER_REV = 112 (28 PPR * 4 quadrature)
 *
 * Optional:
 * - Press X to "zero" and also reset encoder in software.
 * - Press Y to run motor slowly (helps you rotate consistently), press Y again to stop.
 */
@TeleOp(name = "Flywheel Revolution Test", group = "Tests")
public class FlywheelRevolutionTest extends OpMode {

    private DcMotorEx shooter;

    private int startPos = 0;
    private int endPos = 0;
    private boolean hasStart = false;

    private boolean aPrev = false;
    private boolean bPrev = false;
    private boolean xPrev = false;
    private boolean yPrev = false;

    private boolean jogOn = false;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter M1");

        telemetry.addLine("Flywheel Revolution Test ready.");
        telemetry.addLine("A = mark START, B = mark END (delta).");
        telemetry.addLine("X = reset encoder + mark START.");
        telemetry.addLine("Y = toggle slow jog.");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        // X: reset encoder and set start
        if (x && !xPrev) {
            shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            startPos = shooter.getCurrentPosition();
            hasStart = true;
        }
        xPrev = x;

        // A: capture start
        if (a && !aPrev) {
            startPos = shooter.getCurrentPosition();
            hasStart = true;
        }
        aPrev = a;

        // B: capture end and compute delta
        if (b && !bPrev) {
            endPos = shooter.getCurrentPosition();
        }
        bPrev = b;

        // Y: toggle slow jog
        if (y && !yPrev) {
            jogOn = !jogOn;
            shooter.setPower(jogOn ? 0.12 : 0.0); // gentle power
        }
        yPrev = y;

        int curr = shooter.getCurrentPosition();
        int delta = hasStart ? (endPos - startPos) : 0;

        telemetry.addData("Current encoder", curr);
        telemetry.addData("Start encoder", hasStart ? startPos : "Not set (press A or X)");
        telemetry.addData("End encoder", endPos);
        telemetry.addData("Delta (end-start)", hasStart ? delta : "N/A");
        telemetry.addData("Abs delta", hasStart ? Math.abs(delta) : "N/A");
        telemetry.addData("Jog", jogOn ? "ON" : "OFF");
        telemetry.addLine("");
        telemetry.addLine("Instruction: Press A, rotate 1 full turn by hand, press B.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.setPower(0.0);
    }
}
