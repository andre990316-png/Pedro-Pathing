package org.firstinspires.ftc.teamcode.Mechanisms;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Data.FlywheelAndHoodData;

public class FlywheelLogic {


    public static final double kp = 0.39;
    public static final double kd = 0.00002;

    // --- Hardware ---
    private DcMotorEx ShooterM1; // right
    private DcMotorEx ShooterM2; // left
    private Servo ShooterS1;
    private Servo ShooterS2;

    // --- Timing / velocity measurement ---
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime velTimer = new ElapsedTime();
    private int lastPos1 = 0, lastPos2 = 0;

    // IMPORTANT: for Yellow Jacket 6000RPM w/ encoder (common 4x), you used 112 before.
    // Keep it consistent with your drivetrain code.
    private static final double TICKS_PER_REV = 28;

    public boolean isAutoAiming() {
        return autoAiming;
    }

    public void setAutoAiming(boolean autoAiming) {
        this.autoAiming = autoAiming;
    }

    // --- State machine ---
    private enum FlywheelState { IDLE, SPIN_UP, WAIT, LAUNCH, RESET_GATE }
    private FlywheelState flyWheelState = FlywheelState.IDLE;

    // --- Gate / shot settings ---
    private double gateCloseAngle = 0.2;
    private double gateOpenAngle  = 0;
    private double gateOpenTime   = 0.5;
    private double gateCloseTime  = 0.5;

    private int shotsRemaining = 0;

    // --- Velocity targets (RPM) ---
    private double flywheelRpm = 0.0;
    private double minFlywheelRpm = 800;
    private double targetRPM = 0;
    private double flywheelMaxSpinupTime = 2.0;
    private double lastShooterTime=System.nanoTime();
    private IntakeLogic intake = new IntakeLogic();
    private double currentRPM=0;
    private double lastError = 0;
    private double error=0;
    private boolean autoAiming=false;

    public void init(HardwareMap hardwareMap) {
        ShooterM1 = hardwareMap.get(DcMotorEx.class, "Shooter M1");
        ShooterM2 = hardwareMap.get(DcMotorEx.class, "Shooter M2");
        ShooterS1 = hardwareMap.get(Servo.class, "Shooter S1");
        ShooterS2 = hardwareMap.get(Servo.class, "Shooter S2");

        ShooterM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Needed for setVelocity() control
        ShooterM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterM1.setPower(0);
        ShooterM2.setPower(0);
        ShooterS2.setPosition(gateCloseAngle);

        lastPos1 = ShooterM1.getCurrentPosition();
        lastPos2 = ShooterM2.getCurrentPosition();
        velTimer.reset();
        stateTimer.reset();
        flyWheelState = FlywheelState.IDLE;
        intake.init(hardwareMap);
    }

    public void update() {

        long Shooter_now = System.nanoTime();
        double Shooter_dt = (Shooter_now - lastShooterTime) / 1e9;

        if (Shooter_dt <= 0) Shooter_dt = 0.02;

        int pos1 = ShooterM1.getCurrentPosition();

        int dPos1 = pos1 - lastPos1;
        dPos1 = -dPos1;

        double rev1 = dPos1 / TICKS_PER_REV;

        /// current RPM
        currentRPM = (rev1 / Shooter_dt) * 60.0;

        lastPos1 = pos1;
        lastShooterTime = Shooter_now;

        /// convert desired RPM to motor language or something idk
        targetRPM = Range.clip(targetRPM, 0, 6000);

        error = targetRPM - currentRPM;
        double dError = (error - lastError) / Shooter_dt;

        double pdPower = kp * error + kd * dError;

        if (targetRPM <= 0) pdPower = 0;

        double calcRPM = Range.clip(pdPower, 0.0, 1.0);

        lastError = error;


        /// set shooter motors
        ShooterM1.setPower(calcRPM);
        ShooterM2.setPower(-calcRPM);

        intake.update();
        switch (flyWheelState) {
            case IDLE:
                break;
            case WAIT:
                if (stateTimer.seconds() > 0) {
                    stateTimer.reset();
                    flyWheelState = FlywheelState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (flywheelRpm >= minFlywheelRpm || stateTimer.seconds() > flywheelMaxSpinupTime) {
                    ShooterS2.setPosition(gateOpenAngle);
                    stateTimer.reset();
                    flyWheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > gateOpenTime) {
                    shotsRemaining--;
                    ShooterS2.setPosition(gateCloseAngle);
                    stateTimer.reset();
                    flyWheelState = FlywheelState.RESET_GATE;
                }
                break;

            case RESET_GATE:
                if (stateTimer.seconds() > gateCloseTime) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        flyWheelState = FlywheelState.SPIN_UP;
                    } else {
                        ShooterM1.setPower(0);
                        ShooterM2.setPower(0);
                        intake.intakeReady(false);
                        flyWheelState = FlywheelState.IDLE;
                    }
                }
                break;
        }
    }
    public void fireShots(int numberOfShots) {
        if (flyWheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
            if (shotsRemaining > 0) {

                ShooterM1.setPower(targetRPM);
                ShooterM2.setPower(targetRPM);
                intake.intakeReady(true);
                stateTimer.reset();
                flyWheelState = FlywheelState.WAIT;
            }

        }
    }
    public boolean isBusy() {
        return flyWheelState != FlywheelState.IDLE;
    }
    public double getFlywheelRpm() {
        return flywheelRpm;
    }

    public IntakeLogic getIntake(){
        return intake;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    public void setHoodPosition(double hood) {
        // hood is a servo position in [0,1]
        ShooterS1.setPosition(Range.clip(hood, 0.0, 1.0));
    }

    public static class AutoShooting {
        public double rpm;
        public double hood;
        public AutoShooting(double rpm, double hood) {
            this.rpm = rpm;
            this.hood = hood;
        }
    }

    public double getError(){
        return error;
    }

    public double getTargetRPM(){
        return targetRPM;
    }

    public void autoAim(double ta){
        org.firstinspires.ftc.teamcode.Mechanisms.AutoShooting shot;

        shot = (ta >= 0.7) ? FlywheelAndHoodData.lookupA(ta) : FlywheelAndHoodData.lookupB(ta);

        setTargetRPM(shot.rpm);
        setHoodPosition(shot.hood);
    }
}