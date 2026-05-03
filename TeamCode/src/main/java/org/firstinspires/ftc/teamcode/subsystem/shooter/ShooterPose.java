package org.firstinspires.ftc.teamcode.subsystem.shooter;

public class ShooterPose {
    public double x;
    public double y;
    public double z;
    public double yaw;

    public ShooterPose() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.yaw = 0;
    }

    public ShooterPose(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = 0;
    }

    public ShooterPose(double x, double y, double z, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
    }

    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void set(double x, double y, double z, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
    }
}