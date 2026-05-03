package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.op.OpObject;
import org.firstinspires.ftc.teamcode.op.OpStruct;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class Drivetrain extends OpStruct {
    private Follower follower;
    private HardwareMap hardwaremap;
    private final static Drivetrain drivetrain;
    static {
        drivetrain = new Drivetrain();
        OpObject.addStruct(drivetrain);
    }

    private Drivetrain(){}


    public static Follower getFollower(){
        return drivetrain.follower;
    }
    public HardwareMap getHardwaremap(){
        return hardwaremap;
    }


    @Override
    public void Init() {
        follower = Constants.createFollower(m_object.hardwareMap);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {

    }

    @Override
    public void Stop() {
        follower = null;
    }
}
