package org.firstinspires.ftc.teamcode.Data;
import org.firstinspires.ftc.teamcode.Tests.Auto;
import org.firstinspires.ftc.teamcode.Tests.AutoShooting;
import java.lang.Math;

public class FlywheelAndHoodData {
    public static AutoShooting lookupA(double distance) {
        double flywheelRPM, hoodAngle;
        if(distance < 40) {
            flywheelRPM = 3300;
            hoodAngle = 0.84;
        } else if(distance < 70) {
            flywheelRPM = 0.00606061*Math.pow(distance, 4)
                    - 1.41958*Math.pow(distance, 3)
                    + 122.1789*Math.pow(distance, 2)
                    - 4534.42682*distance
                    + 64544.4805;
            hoodAngle = logisticModel(distance);
        } else {
            flywheelRPM = 4400;
            hoodAngle = 1.00;
        }
        return new AutoShooting(flywheelRPM, hoodAngle);
    }
//    public static AutoShooting lookupA(double ta) {
//        if (ta >= 2.37) return new AutoShooting(3500, 0.90);
//        if (ta >= 1.10) return new AutoShooting(4200, 0.94);
//        if (ta >= 0.70) return new AutoShooting(4700, 1.00);
//        return new AutoShooting(4800, 1.00);
//    }

    public static AutoShooting lookupB(double ta) {
        if (ta >= 0.38) return new AutoShooting(5100, 1.00);
        if (ta >= 0.32) return new AutoShooting(5200, 1.00);
        if (ta >= 0.29) return new AutoShooting(5500, 1.00);
        return new AutoShooting(5700, 1.00);
    }
    public static double logisticModel(double x) {
        double A = 1.00125;
        double k = 0.103785;
        double b = 2.41106;

        double z = k * x - b;              // (0.103785x - 2.41106)
        return A / (1.0 + Math.exp(-z));   // A / (1 + e^-z)
    }
}

