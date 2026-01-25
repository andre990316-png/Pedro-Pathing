package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.ArrayList;

public class ColorThing {
    public static int[] colors = {0,0,0}; //0=none, 1=purple, 2=green
    ///  order: colors[0] is the closest artifact to intake, and colors[2] is the one about to be shot
    public static ArrayList<Integer> lastcolors;
    public static int lastcolor=0;
    public static NormalizedColorSensor colorSensor;


    public static void update(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        int color = getColor(colors);
        if(color !=lastcolor){
            push(color);
        }
        lastcolor = color;

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.update();

    }
    public static void init(){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color"); /// <-- CHANGE THE NAME TO THE CORRECT ONE

    }

    public static int getColor(NormalizedRGBA color){
        float[] hsv = new float[3];
        Color.colorToHSV(color.toColor(), hsv);
        if(hsv[1]<0.2){//if saturation too low, probably nothing
            return 0;
        }
        if(color.green<120){//main defining factor between green and purple is the G value
            return 1;
        }
        return 2;
    }
    public static void push(int c){
        colors[2]=colors[1];
        colors[1]=colors[0];
        colors[0]=c;
    }

}
