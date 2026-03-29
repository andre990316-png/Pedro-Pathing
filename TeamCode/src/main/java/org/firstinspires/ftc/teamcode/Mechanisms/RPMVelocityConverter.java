package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RPMVelocityConverter {

    // velocity (m/s) = a * RPM + b
    private double a = 25.0 / 6000.0;  // slope (25 m/s at 6000 RPM)
    private double b = 0;               // intercept

    private double maxRPM = 6000;
    private double maxVelocity = 25;

    public RPMVelocityConverter() {}

    public RPMVelocityConverter(double a, double b) {
        this.a = a;
        this.b = b;
    }

    public double rpmToVelocity(double rpm) {
        if (rpm <= 0) return 0;
        if (rpm >= maxRPM) return maxVelocity;
        return a * rpm + b;
    }

    public double velocityToRPM(double velocity) {
        if (velocity <= 0) return 0;
        if (velocity >= maxVelocity) return maxRPM;
        return (velocity - b) / a;
    }

    // Getters/Setters for Panels tuning
    public double getA() { return a; }
    public void setA(double a) { this.a = a; }
    public double getB() { return b; }
    public void setB(double b) { this.b = b; }
    public double getMaxRPM() { return maxRPM; }
    public void setMaxRPM(double maxRPM) { this.maxRPM = maxRPM; }
    public double getMaxVelocity() { return maxVelocity; }
    public void setMaxVelocity(double maxVelocity) { this.maxVelocity = maxVelocity; }
}