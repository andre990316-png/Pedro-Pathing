package org.firstinspires.ftc.teamcode.Data;
import org.firstinspires.ftc.teamcode.Mechanisms.AutoShooting;

public class FlywheelAndHoodData {
    public static AutoShooting lookupA(double ta) {
        if (ta >= 2.37) return new AutoShooting(3200, 0.90);
        if (ta >= 1.10) return new AutoShooting(3900, 0.94);
        if (ta >= 0.70) return new AutoShooting(4400, 1.00);
        return new AutoShooting(4800, 1.00);
    }

    public static AutoShooting lookupB(double ta) {
        if (ta >= 0.38) return new AutoShooting(4800, 1.00);
        if (ta >= 0.32) return new AutoShooting(4900, 1.00);
        if (ta >= 0.29) return new AutoShooting(5200, 1.00);
        return new AutoShooting(5400, 1.00);
    }
}

