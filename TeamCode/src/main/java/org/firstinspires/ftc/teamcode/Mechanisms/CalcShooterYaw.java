package org.firstinspires.ftc.teamcode.Mechanisms;

public class CalcShooterYaw {
    public static double calcYaw(double x, double y, double tx, double ty, double currheading){
        return Math.atan2(ty-y, tx-x)-currheading;
    }
}
