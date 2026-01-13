package org.firstinspires.ftc.teamcode.Mechanisms;

public class PDCalculation {
    private double kp;
    private double kd;
    private double deadband;
    private double maxPower;
    private double lastTx = 0;
    private double lastTime = 0;
    private double lastPower = 0;
    private double power;
    public PDCalculation (double kp, double kd) {
        this.kp = kp;
        this.kd = kd;
    }
    public void update (double runTimeSeconds, double tx) {
        tx = -tx;
        double dt = runTimeSeconds - lastTime;
        if(dt <= 0) dt = 0.02;
        double dTx = (tx - lastTx) / dt;
        if (Math.abs(tx) <= deadband) {
            power = 0;
        } else {
            power = kp * tx - kd * dTx;
        }
    }
}
