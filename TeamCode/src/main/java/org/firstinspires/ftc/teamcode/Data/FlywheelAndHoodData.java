package org.firstinspires.ftc.teamcode.Data;
import org.firstinspires.ftc.teamcode.Tests.Auto;
import org.firstinspires.ftc.teamcode.Tests.AutoShooting;
import java.lang.Math;

public class FlywheelAndHoodData {
    public static double[] flywheelRanges = new double[] {
            52.5, 55, 57.5, 60, 62.5, 65, 67.5, 70, 72.5, 75, 80, 85, 90, 95, 100, 105, 110, 115, 158
    };

    public static double[] flywheelValues = new double[] {
            2950, 3000, 3150, 3200, 3225, 3250, 3300, 3350, 3450, 3500, 3500, 3600, 3700, 3800, 3900, 3950, 4050, 4150, 4250
    };
    public static double[] hoodRanges = new double[] {
            54.5, 57, 59.5, 62, 64.5, 67, 69.5, 72, 74.5, 77, 82, 87, 92, 97, 100, 105, 110, 115, 158
    };

    public static double[] hoodValues = new double[] {
            0.85, 0.86, 0.87, 0.87, 0.87, 0.87, 0.87, 0.88, 0.89, 0.9, 0.91, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.97, 0.97
    };



    public static AutoShooting lookupA(double distance) {
        double flywheelRPM, hoodAngle;
        flywheelRPM = LinearPointToPointCalculation.calculate(flywheelRanges, flywheelValues, distance);
        hoodAngle = LinearPointToPointCalculation.calculate(hoodRanges, hoodValues, distance);
        /*if(distance < 40) {
            flywheelRPM = 3300;
            hoodAngle = 0.84;
        } else if(distance < 70) {
            flywheelRPM = 0.00606061 * Math.pow(distance, 4)
                    - 1.41958 * Math.pow(distance, 3)
                    + 122.1789 * Math.pow(distance, 2)
                    - 4534.42682 * distance
                    + 64544.4805;
            hoodAngle = logisticModel(distance);
        } else {
            flywheelRPM = 4400;
            hoodAngle = 1.00;
        }*/
        return new AutoShooting(flywheelRPM + 150, hoodAngle + 0.04);
    }
//    public static AutoShooting lookupA(double ta) {
//        if (ta >= 2.37) return new AutoShooting(3500, 0.90);
//        if (ta >= 1.10) return new AutoShooting(4200, 0.94);
//        if (ta >= 0.70) return new AutoShooting(4700, 1.00);
//        return new AutoShooting(4800, 1.00);
//    }

    public static AutoShooting lookupB(double distance) {
        if (distance < 140) return new AutoShooting(4400, 0.99);
        if (distance < 145) return new AutoShooting(4500, 1);
        if (distance < 150) return new AutoShooting(4600, 1);
        if (distance < 155) return new AutoShooting(4700, 1);
        return new AutoShooting(4800, 1);
    }
}

