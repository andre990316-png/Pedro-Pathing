package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.configs.FlyWheelConfig;
import org.firstinspires.ftc.teamcode.utils.configs.IntakeConfig;
import org.firstinspires.ftc.teamcode.utils.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.utils.configs.PinpointConfig;
import org.firstinspires.ftc.teamcode.utils.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.utils.configs.Team;

@Configurable
public class Config {
    public final static MotorConfig motor_lf = new MotorConfig("LF", DcMotorSimple.Direction.FORWARD);
    public final static MotorConfig motor_rf = new MotorConfig("RF", DcMotorSimple.Direction.REVERSE);
    public final static MotorConfig motor_lb = new MotorConfig("LB", DcMotorSimple.Direction.FORWARD);
    public final static MotorConfig motor_rb = new MotorConfig("RB", DcMotorSimple.Direction.REVERSE);

    public final static ServoConfig arm = new ServoConfig("Trig", Servo.Direction.FORWARD);
    public final static ServoConfig aimX1 = new ServoConfig("LH1", Servo.Direction.FORWARD);
    public final static ServoConfig aimX2 = new ServoConfig("LH2", Servo.Direction.FORWARD);
    public final static ServoConfig pitch = new ServoConfig("LV", Servo.Direction.FORWARD);

    public final static IntakeConfig motor_intake1 = new IntakeConfig("IN1", DcMotorSimple.Direction.FORWARD, 1620);

    public final static PIDController flyWheelController1 = new PIDController(4.7,0.07,0.4);
    public final static PIDController flyWheelController2 = new PIDController(4.7,0.07,0.4);
    public final static PIDController intakeController = new PIDController(0, 0, 0);
    public static Team team = Team.None;

    public final static FlyWheelConfig fly_wheel_2 = new FlyWheelConfig(
            "ShooterR",
            DcMotorSimple.Direction.FORWARD,
            6000);
    public final static FlyWheelConfig fly_wheel_1 = new FlyWheelConfig(
            "ShooterL",
            DcMotorSimple.Direction.REVERSE,
            6000);

    public final static PinpointConfig odometry = new PinpointConfig("pinpoint",
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public final static Pose maunalAimingPositionRed  = new Pose(144,140);
    public final static Pose maunalAimingPositionBlue = new Pose(0,140);


}
