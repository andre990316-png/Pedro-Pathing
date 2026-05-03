package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class Drivetrain {
    private Follower follower;
    private HardwareMap hardwaremap;


    public Drivetrain(){
        follower = Constants.createFollower(hardwareMap);
        hardwaremap = hardwareMap;
    }

    public Follower getFollower(){
        return follower;
    }
    public HardwareMap getHardwaremap(){
        return hardwaremap;
    }


}
