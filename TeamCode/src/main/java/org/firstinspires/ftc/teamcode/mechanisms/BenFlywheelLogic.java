/*package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BenFlywheelLogic {

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
    private static final double TICKS_PER_REV = 112.0;

    // --- State machine ---
    private enum FlywheelState { IDLE, SPIN_UP, LAUNCH, RESET_GATE }
    private FlywheelState flyWheelState = FlywheelState.IDLE;

    // --- Gate / shot settings ---
    private double gateCloseAngle = 0.0;
    private double gateOpenAngle  = 0.7;
    private double gateOpenTime   = 0.5;
    private double gateCloseTime  = 0.5;

    private int shotsRemaining = 0;

    // --- Velocity targets (RPM) ---
    private double flywheelRpm = 0.0;          // measured RPM
    private double minFlywheelRpm = 800.0;     // RPM threshold to shoot
    private double targetFlywheelRpm = 1100.0; // desired RPM
    private double flywheelMaxSpinupTime = 2.0;

    private BenIntakeLogic intake = new BenIntakeLogic();


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

    // Convert RPM -> encoder ticks/second for setVelocity()
    private double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    // Update measured flywheel RPM using encoder deltas
    private void updateFlywheelRpm() {
        double dt = velTimer.seconds();
        if (dt <= 0.0) return;

        int p1 = ShooterM1.getCurrentPosition();
        int p2 = ShooterM2.getCurrentPosition();

        int d1 = p1 - lastPos1;
        int d2 = p2 - lastPos2;

        lastPos1 = p1;
        lastPos2 = p2;
        velTimer.reset();

        double tps1 = d1 / dt; // ticks per second
        double tps2 = d2 / dt;

        double rpm1 = (tps1 / TICKS_PER_REV) * 60.0;
        double rpm2 = (tps2 / TICKS_PER_REV) * 60.0;

        // take magnitude + average (since one motor is reversed)
        flywheelRpm = (Math.abs(rpm1) + Math.abs(rpm2)) / 2.0;
    }

    public void update() {
        updateFlywheelRpm(); // <-- keeps flywheelRpm fresh every loop

        switch (flyWheelState) {
            case IDLE:
                if (shotsRemaining > 0) {

                    ShooterM1.setVelocity(rpmToTicksPerSec(targetFlywheelRpm));
                    ShooterM2.setVelocity(-rpmToTicksPerSec(targetFlywheelRpm));

                    stateTimer.reset();
                    flyWheelState = FlywheelState.SPIN_UP;
                    intake.setIntakeOn();

                }
                telemetry.addLine("idle");
                break;

            case SPIN_UP:
                if (flywheelRpm > minFlywheelRpm || stateTimer.seconds() > flywheelMaxSpinupTime) {
                    ShooterS2.setPosition(gateOpenAngle);
                    stateTimer.reset();
                    flyWheelState = FlywheelState.LAUNCH;
                }
                telemetry.addLine("spinup");
                break;

            case LAUNCH:
                if (stateTimer.seconds() > gateOpenTime) {
                    shotsRemaining--;
                    ShooterS2.setPosition(gateCloseAngle);
                    stateTimer.reset();
                    flyWheelState = FlywheelState.RESET_GATE;
                }
                telemetry.addLine("launch");
                break;

            case RESET_GATE:
                if (stateTimer.seconds() > gateCloseTime) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        flyWheelState = FlywheelState.LAUNCH;
                        intake.setIntakeOn();
                    } else {
                        ShooterM1.setPower(0);
                        ShooterM2.setPower(0);
                        flyWheelState = FlywheelState.IDLE;
                        intake.setIntakeOff();
                    }
                }
                telemetry.addLine("resetgate");

                break;
        }
        telemetry.addLine();
        telemetry.update();
    }
    public void fireShots(int numberOfShots) {
        if (flyWheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }
    public boolean isBusy() {
        return flyWheelState != FlywheelState.IDLE;
    }
    public double getFlywheelRpm() {
        return flywheelRpm;
    }


}
*/