package org.firstinspires.ftc.teamcode.Mechanisms;

import static com.pedropathing.math.MathFunctions.clamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeLogic {

    protected DcMotorEx IntakeMotor;
    double iRPM, intake_power;
    double kp = 0.0065, ki = 0, kd = 0.00004;
    public PIDController IntakePID = new PIDController(0, 0, 0);
    static double targetRPM = 0;
    double MaxIntakeRPM = 1650;
//    private enum IntakeState {
//        IDLE,
//        INTAKE
//    }

//    private IntakeState intakeState = IntakeState.IDLE;
    private boolean startIntake = false;

    public static double shootPower = -0.39;

    public void init(HardwareMap hardwareMap) {

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "Intake Motor");

        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setPower(0);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(TelemetryManager telemetryM, Telemetry telemetry) {
        double velocity = targetRPM / MaxIntakeRPM;
        iRPM = -(IntakeMotor.getVelocity() / 28.0) * 60.0;
        double Inorm = iRPM / MaxIntakeRPM;
        IntakePID.setPID(kp, ki, kd);
        intake_power = IntakePID.calculate(Inorm, velocity);
        intake_power = clamp(intake_power + velocity, 0.0, 1.0);

        if(startIntake) {
            IntakeMotor.setPower(intake_power);
        }

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Intake RPM", iRPM);
        telemetry.addData("Intake RPM Error", iRPM - targetRPM);

        telemetryM.addData("iRPM", iRPM);
        telemetryM.addData("targetVelocity", targetRPM);
    }


    public void intakeReady(boolean start) {
        startIntake = start;
    }

//    public boolean isBusy() {
//        return intakeState == IntakeState.INTAKE;
//    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }
}
