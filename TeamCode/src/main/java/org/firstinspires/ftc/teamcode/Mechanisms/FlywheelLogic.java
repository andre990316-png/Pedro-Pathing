package org.firstinspires.ftc.teamcode.Mechanisms;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Data.FlywheelAndHoodData;

public class FlywheelLogic {


    public static final double kp = 2.82;
    public static final double kd = 0.33;
    public static final double kf = 1.666767;


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
    private enum FlywheelState { IDLE, SPIN_UP, LAUNCH, RESET_GATE }
    private FlywheelState flyWheelState = FlywheelState.IDLE;

    // --- Gate / shot settings ---
    private double gateCloseAngle = 1;
    private double gateOpenAngle  = 0.75;
    private double gateOpenTime   = 0.04;
    private double gateCloseTime  = 0.04;

    private int shotsRemaining = 0;

    // --- Velocity targets (RPM) ---
    private double targetRPM = 0;
    private double calcRPM;
    private double flywheelMaxSpinupTime = 0.7;
    private double lastShooterTime=System.nanoTime();
    private IntakeLogic intake = new IntakeLogic();
    private double currentRPM = 0;
    private double lastError = 0;
    private double error = 0;
    private boolean autoAiming = false;
    private boolean swapNextTwoBalls = false;
    private boolean switchAngle = false;
    private Integer swaped = 0;

    public void init(HardwareMap hardwareMap) {
        ShooterM1 = hardwareMap.get(DcMotorEx.class, "Shooter M1");
        ShooterM2 = hardwareMap.get(DcMotorEx.class, "Shooter M2");
        ShooterS1 = hardwareMap.get(Servo.class, "Shooter S1");
        ShooterS2 = hardwareMap.get(Servo.class, "Shooter S2");

        ShooterM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Needed for setVelocity() control
        ShooterM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterM1.setPower(0);
        ShooterM2.setPower(0);
        ShooterS2.setPosition(gateCloseAngle);

        lastPos1 = ShooterM1.getCurrentPosition();
        velTimer.reset();
        stateTimer.reset();
        flyWheelState = FlywheelState.IDLE;
        intake.init(hardwareMap);
    }

    public void update() {
        double ffPower;
        double pPower = kp;
        if(targetRPM < 2000) {
            ffPower = kf * (targetRPM / 6000) * 0.55;  // halve feedforward for very low RPM
        }else if(targetRPM < 3450) {
            ffPower = kf * (targetRPM / 6000) * 0.69;// slightly reduce for mid RPM
            pPower = kp * 0.77;
        } else if (targetRPM < 4500){
            ffPower = kf * (targetRPM / 6000) * 0.7;
            pPower = kp;
        }else if(targetRPM < 5000) {
            ffPower = kf * (targetRPM / 6000) * 0.7;
            pPower = kp;
        }else if (targetRPM < 5500) {
            ffPower = kf * (targetRPM / 6000) * 0.7967;        // full feedforward for high RPM
            pPower = kp;
        } else {
            ffPower = kf * (targetRPM / 6000) * 1.2;
            pPower = kp;
        }


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

        /// pdf cauculation
        targetRPM = Range.clip(targetRPM, 0, 6000);

        error = targetRPM - currentRPM;
        double dError = (error - lastError) / Shooter_dt;

        double pdPower = ffPower + pPower * error / 6000.0 + kd * dError / 6000.0;
        if (targetRPM <= 0) pdPower = 0;

        calcRPM = Range.clip(pdPower, 0.0, 1.0);

        lastError = error;


        /// set shooter motors
        ShooterM1.setPower(calcRPM);
        ShooterM2.setPower(-calcRPM);
        intake.update();

        switch (flyWheelState) {
            case IDLE:
                break;
//            case WAIT:
//                if (stateTimer.seconds() > 0) {
//                    stateTimer.reset();
//                    flyWheelState = FlywheelState.SPIN_UP;
//                }
//                break;
            case SPIN_UP:
                if (Math.abs(error) <= 100 || stateTimer.seconds() > flywheelMaxSpinupTime) {
                    //intake.setIntakeOnVelocity(-0.3);
                    intake.intakeReady(true);
                    ShooterS2.setPosition(gateOpenAngle);
                    stateTimer.reset();
                    flyWheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > gateOpenTime) {
                    shotsRemaining -= 1;
                    intake.intakeReady(false);
                    if (swapNextTwoBalls)
                        swaped++;
                    ShooterS2.setPosition(gateCloseAngle);
                    stateTimer.reset();
                    flyWheelState = FlywheelState.RESET_GATE;
                }
                break;

            case RESET_GATE:
                if (stateTimer.seconds() > gateCloseTime) {
                    if (shotsRemaining > 0) {
                        if (swapNextTwoBalls && swaped == 1)
                            switchAngle = false;
                        else if (swapNextTwoBalls && swaped == 2) {
                            swapNextTwoBalls = false;
                            swaped = 0;
                        }
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
                intake.intakeReady(true);
                stateTimer.reset();
                flyWheelState = FlywheelState.SPIN_UP;
            }

        }
    }
    public boolean isBusy() {
        return flyWheelState != FlywheelState.IDLE;
    }
    public double getFlywheelRpm() {
        return currentRPM;
    }
    public double getCalcRPM() {return calcRPM;}
    public boolean isFlywheelReady() {
        return (Math.abs(error) <= 100);
    }

    public IntakeLogic getIntake(){
        return intake;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    public void setHoodPosition(double hood) {
        // hood is a servo position in [0,1]
        if(swapNextTwoBalls && switchAngle) {
            ShooterS1.setPosition(0);
        } else if (ShooterS1.getPosition() != hood) {
            ShooterS1.setPosition(Range.clip(hood, 0.0, 1.0));
        }
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

    public double getFlywheelPower(){
        return ShooterM1.getPower();
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