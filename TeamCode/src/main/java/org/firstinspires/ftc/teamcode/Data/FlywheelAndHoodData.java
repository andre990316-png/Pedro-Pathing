package org.firstinspires.ftc.teamcode.Data;
import org.firstinspires.ftc.teamcode.Tests.Auto;
import org.firstinspires.ftc.teamcode.Tests.AutoShooting;
import java.lang.Math;

public class FlywheelAndHoodData {
    public static double[] flywheelRanges = new double[] {
            40, 42.5, 45, 47.5, 50, 52.5, 55, 57.5, 60, 62.5, 65, 67.5, 70, 80, 90, 100, 110
    };
    public static double[] flywheelValues = new double[] {
            3300, 3350, 3400, 3525, 3650, 3900, 4050, 4150, 4225, 4275, 4350, 4400, 4405, 4414, 4426, 4438, 4450
    };
    public static double[] hoodRanges = new double[] {
            40, 45, 47.5, 50, 52.5, 55, 57.5, 60, 62.5, 65, 67.5, 70
    };
    public static double[] hoodValues = new double[] {
            0.84, 0.87, 0.89, 0.91, 0.93, 0.945, 0.955, 0.965, 0.975, 0.98, 0.99, 1.0
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
        return new AutoShooting(flywheelRPM, hoodAngle);
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

