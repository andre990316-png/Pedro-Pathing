package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class IntakeLogic {

    private DcMotor IntakeMotor;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime pidTimer = new ElapsedTime();

    private enum IntakeState {
        IDLE,
        INTAKE
    }

    private IntakeState intakeState;
    private boolean startIntake = false;

    public static double intakeOffVelocity = 0;
    public static double intakeOnVelocity = -0.5;

    // PID variables (for display only)
    private double kp = 0.01;
    private double ki = 0.001;
    private double kd = 0.0005;
    private double integral = 0;
    private double lastError = 0;
    private int lastPos = 0;
    private double pidOutput = 0;
    private double currentRPM = 0;
    private double targetRPM = 100;//660

    public void init(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        intakeState = IntakeState.IDLE;
        IntakeMotor.setPower(0);

        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidTimer.reset();
        lastPos = IntakeMotor.getCurrentPosition();
    }

    public void update() {
        // --- original intake logic ---
        switch(intakeState) {
            case IDLE:
                if(startIntake) {
                    IntakeMotor.setPower(intakeOnVelocity);
                    stateTimer.reset();
                    intakeState = IntakeState.INTAKE;
                }
                break;
            case INTAKE:
                if(!startIntake) {
                    IntakeMotor.setPower(intakeOffVelocity);
                    stateTimer.reset();
                    intakeState = IntakeState.IDLE;
                }
                break;
        }

        // --- PID calculation for display only ---
        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt < 0.01) return;

        int pos = IntakeMotor.getCurrentPosition();
        int deltaTicks = pos - lastPos;
        lastPos = pos;

        double ticksPerRev = 145.1;
        double revs = (double) deltaTicks / ticksPerRev;
        currentRPM = Math.abs((revs / dt) * 60.0);

        // PID for debugging (not applied to motor)
        double error = targetRPM - currentRPM;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        pidOutput = kp * error + ki * integral + kd * derivative;
    }

    public void intakeReady(boolean start) {
        startIntake = start;
    }

    public void setIntakeOnVelocity(double p) {
        intakeOnVelocity = Range.clip(p, -1.0, 0.5);
    }

    public void setIntakeOffVelocity(double p) {
        intakeOffVelocity = Range.clip(p, -1.0, 0.5);
    }

    public boolean isBusy() {
        return intakeState != IntakeState.IDLE;
    }

    // --- getters for telemetry/debug ---
    public double getCurrentRPM() {
        return currentRPM;
    }

    public double getPidOutput() {
        return pidOutput;
    }

}
