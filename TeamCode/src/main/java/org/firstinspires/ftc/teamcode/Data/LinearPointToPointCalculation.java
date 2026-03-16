package org.firstinspires.ftc.teamcode.Data;

public class LinearPointToPointCalculation {

    public static double calculate(double[] ranges, double[] values, double val) {
        // Preconditions:
        // ranges.length == values.length
        // ranges is strictly increasing

        int n = ranges.length;
        if (n == 0) return 0;

        if (val <= ranges[0]) return values[0];
        if (val >= ranges[n - 1]) return values[n - 1];

        // Find interval [ranges[i], ranges[i+1]] that contains val
        int i = 0;
        while (i < n - 2 && val > ranges[i + 1]) i++;

        double x0 = ranges[i];
        double x1 = ranges[i + 1];
        double y0 = values[i];
        double y1 = values[i + 1];

        double t = (val - x0) / (x1 - x0);  // 0..1
        return y0 + (y1 - y0) * t;
    }
}
