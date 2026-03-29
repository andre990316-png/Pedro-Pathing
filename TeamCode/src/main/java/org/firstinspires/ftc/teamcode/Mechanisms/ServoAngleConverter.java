package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ServoAngleConverter {

    // angle (degrees) = a * servoPosition + b
    private double a = 30.0;  // 30° at 1.0 position
    private double b = 0;     // 0° at 0 position

    private double minServo = 0.0;
    private double maxServo = 1.0;
    private double minAngle = 0;
    private double maxAngle = 30;

    public double getMinAngle() { return minAngle; }
    public ServoAngleConverter() {}

    public ServoAngleConverter(double a, double b) {
        this.a = a;
        this.b = b;
    }

    public double servoToAngle(double position) {
        if (position <= minServo) return minAngle;
        if (position >= maxServo) return maxAngle;
        return a * position + b;
    }

    public double angleToServo(double angle) {
        if (angle <= minAngle) return minServo;
        if (angle >= maxAngle) return maxServo;
        return (angle - b) / a  ;
    }

    // Getters/Setters for Panels tuning
    public double getA() { return a; }
    public void setA(double a) { this.a = a; }
    public double getB() { return b; }
    public void setB(double b) { this.b = b; }
    public double getMaxAngle() { return maxAngle; }
    public void setMaxAngle(double maxAngle) { this.maxAngle = maxAngle; }
}