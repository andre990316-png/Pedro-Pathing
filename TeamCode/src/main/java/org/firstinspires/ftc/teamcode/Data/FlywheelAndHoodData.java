package org.firstinspires.ftc.teamcode.Data;
import org.firstinspires.ftc.teamcode.Tests.AutoShooting;

public class FlywheelAndHoodData {
    public static AutoShooting lookupA(double ta) {
        if (ta >= 2.37) return new AutoShooting(3500, 0.90);
        if (ta >= 1.10) return new AutoShooting(4200, 0.94);
        if (ta >= 0.70) return new AutoShooting(4700, 1.00);
        return new AutoShooting(4800, 1.00);
    }

    public static AutoShooting lookupB(double ta) {
        if (ta >= 0.38) return new AutoShooting(5100, 1.00);
        if (ta >= 0.32) return new AutoShooting(5200, 1.00);
        if (ta >= 0.29) return new AutoShooting(5500, 1.00);
        return new AutoShooting(5700, 1.00);
    }
}

