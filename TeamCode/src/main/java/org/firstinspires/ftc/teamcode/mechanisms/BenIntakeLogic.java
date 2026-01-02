/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BenIntakeLogic {
    private DcMotor IntakeMotor;
    private double intakeOffVelocity = 0;
    private double intakeOnVelocity = 1;

    private boolean intakeOn = false;


    public void init(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        IntakeMotor.setPower(0);
    }
    public void update() {
        if(intakeOn){
            IntakeMotor.setPower(intakeOnVelocity);
        }else{
            IntakeMotor.setPower(intakeOffVelocity);
        }
    }
    public boolean isIntakeOn() {
        return intakeOn;
    }
    public void setIntakeOn() {
        intakeOn = true;
    }
    public void setIntakeOff() {
        intakeOn = false;
    }




}*/