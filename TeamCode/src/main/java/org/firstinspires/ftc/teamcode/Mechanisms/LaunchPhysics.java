package org.firstinspires.ftc.teamcode.Mechanisms;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LaunchPhysics {

    private static final double GRAVITY = 9.81;  // m/s²
    private static final double INCH_TO_METER = 0.0254;

    private double launchHeight = 20;     // inches
    private double targetHeight = 98;     // inches
    private double contactAngle = -30;    // degrees (negative = descending)

    public LaunchPhysics() {}

    /**
     * Calculate required velocity and angle to hit target
     */
    public Result calculate(double launchX, double launchY, double targetX, double targetY) {
        // Convert to meters
        double launchX_m = launchX * INCH_TO_METER;
        double launchY_m = launchY * INCH_TO_METER;
        double targetX_m = targetX * INCH_TO_METER;
        double targetY_m = targetY * INCH_TO_METER;

        // Distance and height
        double dx = targetX_m - launchX_m;
        double dy = targetY_m - launchY_m;
        double distance = Math.hypot(dx, dy);
        double deltaZ = (targetHeight - launchHeight) * INCH_TO_METER;

        // Time of flight from contact angle
        double tanPhi = Math.tan(Math.toRadians(contactAngle));
        double flightTime = Math.sqrt(2 * (distance * tanPhi - deltaZ) / GRAVITY);

        // Launch velocity components
        double vCosTheta = distance / flightTime;
        double vSinTheta = (deltaZ + 0.5 * GRAVITY * flightTime * flightTime) / flightTime;

        // Results
        double velocity = Math.hypot(vCosTheta, vSinTheta);
        double angle = Math.toDegrees(Math.atan2(vSinTheta, vCosTheta));

        return new Result(velocity, angle, flightTime);
    }

    /**
     * Calculate launch angle given a fixed velocity
     * Uses projectile motion equations
     */
    public double calculateAngleFromVelocity(
            double launchX, double launchY,
            double targetX, double targetY,
            double velocity) {

        double dx = (targetX - launchX) * INCH_TO_METER;
        double dy = (targetY - launchY) * INCH_TO_METER;
        double distance = Math.hypot(dx, dy);
        double deltaZ = (targetHeight - launchHeight) * INCH_TO_METER;

        double v2 = velocity * velocity;
        double g = GRAVITY;

        double inside = v2 * v2 - g * (g * distance * distance + 2 * deltaZ * v2);
        if (inside < 0) inside = 0; // avoid NaN if velocity too low

        double angle = Math.atan((v2 - Math.sqrt(inside)) / (g * distance));
        return Math.toDegrees(angle);
    }

    public void setLaunchHeight(double inches) { this.launchHeight = inches; }
    public void setTargetHeight(double inches) { this.targetHeight = inches; }
    public void setContactAngle(double degrees) { this.contactAngle = degrees; }

    public static class Result {
        public final double velocity;   // m/s
        public final double angle;      // degrees
        public final double flightTime; // seconds

        public Result(double velocity, double angle, double flightTime) {
            this.velocity = velocity;
            this.angle = angle;
            this.flightTime = flightTime;
        }
    }
}