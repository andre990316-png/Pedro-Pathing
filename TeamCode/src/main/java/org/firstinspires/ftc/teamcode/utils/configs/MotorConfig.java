package org.firstinspires.ftc.teamcode.utils.configs;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorConfig {
    public MotorConfig(String id, DcMotorSimple.Direction direction) {
        this.id = id;
        this.direction = direction;
    }

    public final String id;
    public final DcMotorSimple.Direction direction;
}
