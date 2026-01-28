package org.firstinspires.ftc.teamcode.Data;
import org.firstinspires.ftc.teamcode.Tests.Auto;
import org.firstinspires.ftc.teamcode.Tests.AutoShooting;
import java.lang.Math;

public class FlywheelAndHoodData {
    public static double[] flywheelRanges = new double[] {
            52.5, 55, 57.5, 60, 62.5, 65, 67.5, 70, 72.5, 75, 80, 85, 90, 95, 100, 105, 110
    };

    public static double[] flywheelValues = new double[] {
            3250, 3250, 3275, 3300, 3325, 3350, 3400, 3450, 3550, 3600, 3600, 3700, 3800, 3900, 4000, 4050, 4150
    };
    public static double[] hoodRanges = new double[] {
            52.5, 55, 57.5, 60, 62.5, 65, 67.5, 70, 72.5, 75, 80, 85, 90, 95, 100, 105, 110
    };

    public static double[] hoodValues = new double[] {
            0.88, 0.89, 0.90, 0.90, 0.90, 0.90, 0.90, 0.91, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.98, 0.99, 1.00
    };



    public static AutoShooting lookupA(double distance) {
        double flywheelRPM, hoodAngle;
        flywheelRPM = CalcStuff.iwrotethiswithoutchatgptyouguysarenooooobs(flywheelRanges, flywheelValues, distance);
        hoodAngle = CalcStuff.iwrotethiswithoutchatgptyouguysarenooooobs(hoodRanges, hoodValues, distance);
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
        return new AutoShooting(flywheelRPM - 50, hoodAngle);
    }
//    public static AutoShooting lookupA(double ta) {
//        if (ta >= 2.37) return new AutoShooting(3500, 0.90);
//        if (ta >= 1.10) return new AutoShooting(4200, 0.94);
//        if (ta >= 0.70) return new AutoShooting(4700, 1.00);
//        return new AutoShooting(4800, 1.00);
//    }

    public static AutoShooting lookupB(double ta) {
        double flywheelRPM, hoodAngle = 1;
        if (ta >= 0.38) return new AutoShooting(5100, 1.00);
        if (ta >= 0.32) return new AutoShooting(5200, 1.00);
        if (ta >= 0.29) return new AutoShooting(5500, 1.00);
        return new AutoShooting(5700, 1.00);
    }
    public static double logisticModel(double x) {
        return 1.04906 / (1.0 + Math.exp(-(0.0533548 * x - 0.774792)));
    }
}

