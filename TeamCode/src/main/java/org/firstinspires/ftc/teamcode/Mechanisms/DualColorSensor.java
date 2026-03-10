package org.firstinspires.ftc.teamcode.Mechanisms;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DualColorSensor {
    private final NormalizedColorSensor s1;
    private final NormalizedColorSensor s2;

    private final float[] hsv1 = new float[3];
    private final float[] hsv2 = new float[3];

    private NormalizedRGBA c1 = new NormalizedRGBA();
    private NormalizedRGBA c2 = new NormalizedRGBA();

    private int lastColor1 = 0;
    private int lastColor2 = 0;
    private int fusedColor = 0;

    public DualColorSensor(HardwareMap hw, String name1, String name2, float gain) {
        s1 = hw.get(NormalizedColorSensor.class, name1);
        s2 = hw.get(NormalizedColorSensor.class, name2);
        s1.setGain(gain);
        s2.setGain(gain);
    }

    public void update() {
        c1 = s1.getNormalizedColors();
        c2 = s2.getNormalizedColors();

        Color.colorToHSV(c1.toColor(), hsv1);
        Color.colorToHSV(c2.toColor(), hsv2);

        lastColor1 = classifyHSV(c1, hsv1);
        lastColor2 = classifyHSV(c2, hsv2);
        fusedColor = fuse(lastColor1, lastColor2);
    }

    public int getBallColor() {
        return fusedColor;
    }

    public int getSensor1Color() {
        return lastColor1;
    }

    public int getSensor2Color() {
        return lastColor2;
    }

    public NormalizedRGBA getRaw1() {
        return c1;
    }

    public NormalizedRGBA getRaw2() {
        return c2;
    }

    public float[] getHsv1() {
        return hsv1;
    }

    public float[] getHsv2() {
        return hsv2;
    }

    private static int classifyHSV(NormalizedRGBA c, float[] hsv) {
        float H = hsv[0];
        float S = hsv[1];

        double intensity = c.red + c.green + c.blue;

        if (intensity < 0.02 || S < 0.15) return 0;

        if (H >= 140 && H <= 170) return 1; // green (or swap if you want)
        if (H >= 180 && H <= 240) return 2; // purple

        return 0;
    }

    private static int fuse(int a, int b) {
        if (a == 0 && b == 0) return 0;
        if (a == b) return a;
        if (a == 0) return b;
        if (b == 0) return a;
        return 0;
    }

    public void telemetry(Telemetry t, String label) {
        t.addData(label + " ball", fusedColor);

        t.addLine(label + " 1")
                .addData("RGB", "%.3f %.3f %.3f", c1.red, c1.green, c1.blue)
                .addData("HSV", "H=%.1f S=%.2f V=%.2f", hsv1[0], hsv1[1], hsv1[2])
                .addData("C", lastColor1);

        t.addLine(label + " 2")
                .addData("RGB", "%.3f %.3f %.3f", c2.red, c2.green, c2.blue)
                .addData("HSV", "H=%.1f S=%.2f V=%.2f", hsv2[0], hsv2[1], hsv2[2])
                .addData("C", lastColor2);
    }

//    public void telemetry(TelemetryManager tm, String label) {
//        if (tm == null) return;
//
//        tm.debug(label + "_ball", fusedColor);
//
//        tm.debug(label + "_1_r", c1.red);
//        tm.debug(label + "_1_g", c1.green);
//        tm.debug(label + "_1_b", c1.blue);
//        tm.debug(label + "_1_h", hsv1[0]);
//        tm.debug(label + "_1_s", hsv1[1]);
//        tm.debug(label + "_1_v", hsv1[2]);
//        tm.debug(label + "_1_c", lastColor1);
//
//        tm.debug(label + "_2_r", c2.red);
//        tm.debug(label + "_2_g", c2.green);
//        tm.debug(label + "_2_b", c2.blue);
//        tm.debug(label + "_2_h", hsv2[0]);
//        tm.debug(label + "_2_s", hsv2[1]);
//        tm.debug(label + "_2_v", hsv2[2]);
//        tm.debug(label + "_2_c", lastColor2);
//    }

//    public static int getColor(NormalizedRGBA color){
//        float[] hsv = new float[3];
//        Color.colorToHSV(color.toColor(), hsv);
//        if(hsv[1] < 0.2){//if saturation too low, probably nothing
//            return 0;
//        }
//        if((color.green + color.blue > 0.02) && (color.green < color.blue)){//main defining factor between green and purple is the G value
//            return 1;
//        }
//        return 2;
//    }
}