package org.firstinspires.ftc.teamcode.Mechanisms;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Data.FlywheelAndHoodData;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
@Configurable
public class FlywheelLogic {
    protected DcMotorEx shooterL, shooterR;
    double rRPM, lRPM, shooterR_power, shooterL_power;
    public PIDController ShooterRPID = new PIDController(0, 0, 0);
    public PIDController ShooterLPID = new PIDController(0, 0, 0);
    public static final double kp = 2.82, ki = 0.25, kd = 0.33, kf = 1.5;
    static double rP = 2.82, rI = 0.25, rD = 0.33, lP = 5, lI = 0.25, lD = 0.33;
    static double targetRPM = 0;
    double MaxLeftRPM = 6000, MaxRightPM = 6000;
    private Servo HoodServo, TriggerServo;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // --- State machine ---
    private enum FlywheelState { IDLE, SPIN_UP, LAUNCH}
    private FlywheelState flyWheelState = FlywheelState.IDLE;

    // --- Gate / shot settings ---
    private double gateCloseAngle = 0.575;
    private double gateOpenAngle  = 0.527;
    private double launchTime = 1.7; // seconds gate stays open
    private int shotsRemaining = 0;
    private double singleShotTime = 2;

    // --- Velocity targets (RPM) ---
    private double flywheelMaxSpinupTime = 0.7;
    private double currentRPM = 0;
    private double error = 0;
    private boolean swapNextTwoBalls = false;
    private boolean switchAngle = false;
    private Integer swaped = 0;

    public void init(HardwareMap hardwareMap) {
        shooterL = hardwareMap.get(DcMotorEx.class, "Shooter M1");
        shooterR = hardwareMap.get(DcMotorEx.class, "Shooter M2");
        HoodServo = hardwareMap.get(Servo.class, "Shooter S1");
        TriggerServo = hardwareMap.get(Servo.class, "Shooter S2");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setPower(0);
        shooterR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setPower(0);
        TriggerServo.setPosition(gateCloseAngle);
        stateTimer.reset();
        flyWheelState = FlywheelState.IDLE;
    }

    public void update(IntakeLogic intake, Telemetry telemetry, TelemetryManager telemetryM) {
        double velocity1 = targetRPM / MaxRightPM;
        double velocity2 = targetRPM / MaxLeftRPM;
        rRPM = -(shooterL.getVelocity() / 28.0) * 60.0;
        lRPM = -(shooterL.getVelocity() / 28.0) * 60.0;

//        double rNorm = rRPM / MaxRightPM;
//        ShooterRPID.setPID(rP, rI, rD); // setting PID
//        shooterR_power = ShooterRPID.calculate(rNorm, velocity1); // calculate output power
        double lNorm = lRPM / MaxLeftRPM;
        ShooterLPID.setPID(lP, lI, lD); // setting PID
        shooterL_power = ShooterLPID.calculate(lNorm, velocity2); // calculate output power
        shooterR_power = shooterL_power;
        shooterR_power = clamp(shooterR_power + velocity1, -1, 1.0); // setting in correct range
        shooterL_power = clamp(shooterL_power + velocity2, -1, 1.0);
        shooterR.setPower(shooterR_power);
        shooterL.setPower(shooterL_power);

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Right RPM", rRPM);
        telemetry.addData("Left RPM", lRPM);
        telemetry.addData("Right RPM Error", rRPM - targetRPM);
        telemetry.addData("Left RPM Error", lRPM - targetRPM);

        telemetryM.addData("rRPM", rRPM);
        telemetryM.addData("lRPM", lRPM);
        telemetryM.addData("targetVelocity", targetRPM);
//        telemetryM.update();

        switch (flyWheelState) {
            case IDLE:
                break;

                case SPIN_UP:
                    if ((Math.abs(rRPM - targetRPM) <= 100 && Math.abs(lRPM - targetRPM) <= 100)  || stateTimer.seconds() > flywheelMaxSpinupTime) {
                        TriggerServo.setPosition(gateOpenAngle);
                        intake.setTargetRPM(-0.39);
                        intake.intakeReady(true);
                        stateTimer.reset();
                        flyWheelState = FlywheelState.LAUNCH;
                    }
                    break;

                case LAUNCH:
                    if (stateTimer.seconds() > singleShotTime) {
                        shotsRemaining--;
                        if (shotsRemaining > 0) {
                            intake.setTargetRPM(-0.39);
                            intake.intakeReady(true);
                            stateTimer.reset();
                            flyWheelState = FlywheelState.SPIN_UP;
                        } else {
                            shooterL.setPower(0);
                            shooterR.setPower(0);
                            intake.intakeReady(false);
                            TriggerServo.setPosition(gateCloseAngle);
                            flyWheelState = FlywheelState.IDLE;
                        }
                    }
                    break;


//            case RESET_GATE:
//                if (stateTimer.seconds() > gateCloseTime) {
//                    if (shotsRemaining > 0) {
//                        if (swapNextTwoBalls && swaped == 1)
//                            switchAngle = false;
//                        else if (swapNextTwoBalls && swaped == 2) {
//                            swapNextTwoBalls = false;
//                            swaped = 0;
//                        }
//                        stateTimer.reset();
//                        flyWheelState = FlywheelState.SPIN_UP;
//                    } else {
//                        shooterL.setPower(0);
//                        shooterR.setPower(0);
//                        intake.intakeReady(false);
//                        intake.setIntakeOnVelocity(-1);
//                        flyWheelState = FlywheelState.IDLE;
//                    }
//                }
//                break;
        }
    }
    public void fireShots(int numberOfShots) {
        if (flyWheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
            singleShotTime = launchTime / numberOfShots;

            stateTimer.reset();
            flyWheelState = FlywheelState.SPIN_UP;
        }
    }
    public boolean isBusy() {
        return flyWheelState != FlywheelState.IDLE;
    }
    public Servo getHoodServo() {
        return HoodServo;
    }
    public double geLRPM() {
        return lRPM;
    }
    public double getRRPM() {
        return rRPM;
    }
    public boolean isFlywheelReady() {
        return (Math.abs(rRPM - targetRPM) <= 100 && Math.abs(lRPM - targetRPM) <= 100);
    }
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }
    public double getTargetRPM(){
        return targetRPM;
    }
    public void setHoodPosition(double hood) {
        if(swapNextTwoBalls && switchAngle) {
            HoodServo.setPosition(0);
        } else if (HoodServo.getPosition() != hood) {
            HoodServo.setPosition(Range.clip(hood, 0.0, 1.0));
        }
    }
    public void setSwapNextTwoBalls() {
        if(shotsRemaining >= 2) {
            switchAngle = true;
            swaped = 0;
            swapNextTwoBalls = true;
        }
    }
    public void autoAim(double distToGoal){
        org.firstinspires.ftc.teamcode.Tests.AutoShooting shot;

        shot = (distToGoal < 125) ? FlywheelAndHoodData.lookupA(distToGoal) : FlywheelAndHoodData.lookupB(distToGoal);
        setTargetRPM(shot.rpm);
        setHoodPosition(shot.hood);
    }
}