package org.firstinspires.ftc.teamcode.utils.configs;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FlyWheelConfig extends MotorConfig{
    public FlyWheelConfig(String id, DcMotorSimple.Direction direction, double maxVelocity) {
        super(id, direction);
        this.maxVelocity = maxVelocity;
    }
    public final double maxVelocity;
}
