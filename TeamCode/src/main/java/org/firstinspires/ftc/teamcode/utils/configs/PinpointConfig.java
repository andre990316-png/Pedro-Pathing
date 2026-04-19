package org.firstinspires.ftc.teamcode.utils.configs;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class PinpointConfig {
    public PinpointConfig(String id, GoBildaPinpointDriver.EncoderDirection forward_direction, GoBildaPinpointDriver.EncoderDirection strafe_direction) {
        this.id = id;
        this.forward_direction = forward_direction;
        this.strafe_direction  = strafe_direction;
    }

    public final String id;
    public final GoBildaPinpointDriver.EncoderDirection forward_direction;
    public final GoBildaPinpointDriver.EncoderDirection strafe_direction;
}
