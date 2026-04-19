package org.firstinspires.ftc.teamcode.utils.configs;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeConfig extends MotorConfig{
    public IntakeConfig(String id, DcMotorSimple.Direction direction, double maxVelocity) {
        super(id, direction);
        this.maxVelocity = maxVelocity;
    }
    public final double maxVelocity;
}
