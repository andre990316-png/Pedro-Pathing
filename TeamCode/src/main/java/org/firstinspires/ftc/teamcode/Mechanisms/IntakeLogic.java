package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class IntakeLogic {

    private DcMotor IntakeMotor;

    private ElapsedTime pidTimer = new ElapsedTime();

    private enum IntakeState {
        IDLE,
        INTAKE
    }

    private IntakeState intakeState = IntakeState.IDLE;
    private boolean startIntake = false;

    // PID tuning (start here)
    private double kp = 0.0025;
    private double ki = 0.0004;
    private double kd = 0.00015;

    private double integral = 0;
    private double lastError = 0;
    private double pidPower = 0;

    private int lastPos = 0;

    private double currentRPM = 0;
    private double targetRPM = 0;

    // GoBilda 5203 encoder
    private static final double TICKS_PER_REV = 145.1;

    public void init(HardwareMap hardwareMap) {

        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");

        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidTimer.reset();
        lastPos = IntakeMotor.getCurrentPosition();

        IntakeMotor.setPower(0);
    }

    public void update() {

        if (startIntake) {
            intakeState = IntakeState.INTAKE;
        } else {
            intakeState = IntakeState.IDLE;
        }

        if (intakeState == IntakeState.INTAKE) {


            double dt = pidTimer.seconds();
            pidTimer.reset();

            if (dt < 0.01) return;

            int pos = IntakeMotor.getCurrentPosition();
            int deltaTicks = pos - lastPos;
            lastPos = pos;

            double revs = deltaTicks / TICKS_PER_REV;
            currentRPM = (revs / dt) * 60.0;

            double error = targetRPM - currentRPM;

            integral += error * dt;
            integral = Range.clip(integral, -500, 500);

            double derivative = (error - lastError) / dt;
            lastError = error;

            pidPower = kp * error + ki * integral + kd * derivative;
            pidPower = Range.clip(pidPower, -1.0, 1.0);

            IntakeMotor.setPower(pidPower);

        } else {

            // IDLE
            IntakeMotor.setPower(0);
            pidPower = 0;
            integral = 0;
            lastError = 0;
            currentRPM = 0;
        }
    }
    public void intakeReady(boolean start) {
        startIntake = start;
        if (!start) {
            targetRPM = 0;
            integral = 0;
            pidPower = 0;
        }
    }

    public boolean isBusy() {
        return intakeState == IntakeState.INTAKE;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getPidPower() {
        return pidPower;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

}
