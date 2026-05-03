package org.firstinspires.ftc.teamcode.run;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.mechanisms.ButtonLogic;
import org.firstinspires.ftc.teamcode.op.OpObject;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

import java.util.ArrayList;

public class TeleOp extends OpObject {

    /// initialize variables here
    /// motors, etc
    Drivetrain drivetrain;
    Follower follower;
    ArrayList<ButtonLogic> allButtons;

    @Override
    public void Init() {
        /// init stuff here
        drivetrain = new Drivetrain();
        follower = drivetrain.getFollower();
        allButtons = new ArrayList<>();



    }

    @Override
    public void Start() {
        /// start stuff here
        follower.startTeleOpDrive();
        follower.setPose(new Pose(72, 72, Math.toRadians(90))); //TODO make pose class
    }

    @Override
    public void Loop() {

    }

    @Override
    public void Stop() {

    }
}
