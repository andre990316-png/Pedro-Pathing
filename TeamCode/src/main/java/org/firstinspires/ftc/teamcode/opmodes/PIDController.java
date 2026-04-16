package org.firstinspires.ftc.teamcode.opmodes;

public class PIDController {

    double kP, kI, kD;
    double integral = 0;
    double lastError = 0;

    public PIDController(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public double update(double target, double current) {
        double error = target - current;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }
}