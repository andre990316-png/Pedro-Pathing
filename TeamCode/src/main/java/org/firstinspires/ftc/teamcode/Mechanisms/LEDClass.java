package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class LEDClass {
    private LED redLED;
    private LED greenLED;

    public void init(HardwareMap hardwareMap, int i) {
        String num = String.valueOf(i);
        greenLED = hardwareMap.get(LED.class, "LED"+num+"Green");
        redLED = hardwareMap.get(LED.class, "LED"+num+"Red");
    }
    public void setRedLED(boolean isON) {
        if(isON) redLED.on();
        else redLED.off();
    }
    public void setGreenLED(boolean isON) {
        if(isON) greenLED.on();
        else greenLED.off();
    }
}
