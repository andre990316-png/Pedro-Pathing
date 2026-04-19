package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.utils.configs.PinpointConfig;

@Configurable
public class Config {
    public final static MotorConfig motor_lf = new MotorConfig("LF", DcMotorSimple.Direction.FORWARD);
    public final static MotorConfig motor_rf = new MotorConfig("RF", DcMotorSimple.Direction.REVERSE);
    public final static MotorConfig motor_lb = new MotorConfig("LB", DcMotorSimple.Direction.FORWARD);
    public final static MotorConfig motor_rb = new MotorConfig("RB", DcMotorSimple.Direction.REVERSE);

    public final static PinpointConfig odometry = new PinpointConfig("pinpoint",
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.FORWARD);
}
