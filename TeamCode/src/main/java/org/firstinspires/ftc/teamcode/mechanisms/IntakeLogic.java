package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeLogic {
    private DcMotor IntakeMotor;
    private ElapsedTime stateTimer = new ElapsedTime();
    private enum IntakeState {
        IDLE,
        INTAKE
    }
    private IntakeState intakeState;
    private boolean startIntake = false;
    private double intakeOffVelocity = 0;
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
        if(intakeState == IntakeState.IDLE) {
            startIntake = start;
        }
    }
    public boolean isBusy() {
        return intakeState != IntakeState.IDLE;
    }
}
