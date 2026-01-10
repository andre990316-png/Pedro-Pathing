package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;


public class AutoStep {
    public Pose pose;              // null means "no movement step" (pause / wait step)
    public Auto_V2.AutoAction action;
    public long valueMs;           // used by PAUSE_MS

    public AutoStep(Pose pose, Auto_V2.AutoAction action, long valueMs) {
        this.pose = pose;
        this.action = action;
        this.valueMs = valueMs;
    }

    public Pose getPose(){
        return pose;
    }
    public void setPose(Pose p){
        pose = p;
    }
    public Auto_V2.AutoAction getAction(){
        return action;
    }
    public long getValueMs(){
        return valueMs;
    }

    public static ArrayList<AutoStep> flipped(ArrayList<AutoStep> arr){
        ArrayList<AutoStep> ret = new ArrayList<>();
        for(AutoStep a:arr){
            Pose p = new Pose(144-a.getPose().getX(), a.getPose().getY(), Math.PI-a.getPose().getHeading());
            //flips x coordinate along x=72, and flips heading along PI/2 radians
            ret.add(new AutoStep(p, a.getAction(), a.getValueMs()));
        }
        return ret;
    }
}
