package org.firstinspires.ftc.teamcode.Mechanisms;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SingleColorSensor {
    private final NormalizedColorSensor s;

    private final float[] hsv = new float[3];

    private NormalizedRGBA c = new NormalizedRGBA();

    private int lastColor1 = 0;

    public SingleColorSensor(HardwareMap hw, String name1, float gain) {
        s = hw.get(NormalizedColorSensor.class, name1);
        s.setGain(gain);
    }

    public void update() {
        c = s.getNormalizedColors();

        Color.colorToHSV(c.toColor(), hsv);

        lastColor1 = classifyHSV(c, hsv);
    }

    public int getBallColor() {
        return lastColor1;
    }


    public NormalizedRGBA getRaw1() {
        return c;
    }

    public float[] getHsv() {
        return hsv;
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

    public void telemetry(Telemetry t, String label) {
        t.addData(label + " ball", lastColor1);

        t.addLine(label + " 1")
                .addData("RGB", "%.3f %.3f %.3f", c.red, c.green, c.blue)
                .addData("HSV", "H=%.1f S=%.2f V=%.2f", hsv[0], hsv[1], hsv[2])
                .addData("C", lastColor1);
    }
}