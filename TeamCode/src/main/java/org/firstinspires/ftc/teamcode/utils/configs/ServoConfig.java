package org.firstinspires.ftc.teamcode.utils.configs;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoConfig {
    public ServoConfig(String id, Servo.Direction direction) {
        this.id = id;
        this.direction = direction;
    }

    public final String id;
    public final Servo.Direction direction;
}
