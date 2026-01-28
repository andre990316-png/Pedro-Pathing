package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class IntakeLogic {
    private DcMotor IntakeMotor;
    private ElapsedTime stateTimer = new ElapsedTime();
    private enum IntakeState {
        IDLE,
        INTAKE
    }
    private IntakeState intakeState;
    private boolean startIntake = false;
    private double intakeOffVelocity = -0.3;
    private double intakeOnVelocity = -1;
    public void init(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        intakeState = IntakeState.IDLE;
        IntakeMotor.setPower(0);
    }
    public void update() {
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
}
