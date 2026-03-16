package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class CalcShooterYaw {
    public static double calcYaw(double x, double y, double tx, double ty, double currheading){
        return Math.atan2(ty-y, tx-x)-currheading;
    }
}
