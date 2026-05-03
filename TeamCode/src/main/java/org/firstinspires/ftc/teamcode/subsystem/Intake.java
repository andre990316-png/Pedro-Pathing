package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.subsystem.shooter.ShooterCalculator.TicksPerSecToRPM;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.op.OpObject;
import org.firstinspires.ftc.teamcode.op.OpStruct;
import org.firstinspires.ftc.teamcode.subsystem.shooter.ShooterCalculator;
import org.firstinspires.ftc.teamcode.utils.Config;
import org.firstinspires.ftc.teamcode.utils.configs.IntakeConfig;


public class Intake extends OpStruct {
    private Intake(){}

    private final static Intake intake;
    static {
        intake = new Intake();
        OpObject.addStruct(intake);
    }

    public static DcMotorEx intake1;

    public static double targetRPM = 0;
    public static double actualRPM = 0;

    public static void in()  { targetRPM = 1620;  }
    public static void out() { targetRPM = -1620; }
    public static void off() { targetRPM = 0;  }

    private final PIDController m_intakeController = Config.intakeController;


    @Override
    public void Init() {
        intake1 = init_intake(Config.motor_intake1);
    }

    @Override
    public void Start() {
        off();
    }

    @Override
    public void Loop() {
        actualRPM = TicksPerSecToRPM(intake1.getVelocity());
        update_intake(intake1, m_intakeController, targetRPM, Config.motor_intake1.maxVelocity);

    }

    @Override
    public void Stop() {
        off();
    }


    private void update_intake(DcMotorEx motor, PIDController controller, double targetRPM, double maxRPM) {
        final double targetPower  = targetRPM / maxRPM;
        final double currentRPM   = TicksPerSecToRPM(intake1.getVelocity());
        final double currentPower = currentRPM / maxRPM;
        final double newPower     = clamp(
                controller.calculate(currentPower, targetPower) + targetPower, -1.0, 1.0);
        motor.setPower(newPower);
    }

    private DcMotorEx init_intake(IntakeConfig config) {
        DcMotorEx ret = m_object.hardwareMap.get(DcMotorEx.class, config.id);
        ret.setDirection(config.direction);
        ret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ret.setPower(0);
        return ret;
    }
}
