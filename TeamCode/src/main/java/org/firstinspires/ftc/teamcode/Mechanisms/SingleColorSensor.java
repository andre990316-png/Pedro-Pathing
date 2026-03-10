package org.firstinspires.ftc.teamcode.Mechanisms;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SingleColorSensor {
    private final NormalizedColorSensor s1;

    private final float[] hsv1 = new float[3];
    private final float[] hsv2 = new float[3];

    private NormalizedRGBA c1 = new NormalizedRGBA();
    private NormalizedRGBA c2 = new NormalizedRGBA();

    private int lastColor1 = 0;
    private int lastColor2 = 0;
    private int fusedColor = 0;

    public SingleColorSensor(HardwareMap hw, String name1, float gain) {
        s1 = hw.get(NormalizedColorSensor.class, name1);
        s1.setGain(gain);
    }

    public void update() {
        c1 = s1.getNormalizedColors();

        Color.colorToHSV(c1.toColor(), hsv1);

        lastColor1 = classifyHSV(c1, hsv1);
        fusedColor = lastColor1;
    }

    public int getBallColor() {
        return fusedColor;
    }

    public int getSensor1Color() {
        return lastColor1;
    }

    public NormalizedRGBA getRaw1() {
        return c1;
    }

    public float[] getHsv1() {
        return hsv1;
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
    }
}