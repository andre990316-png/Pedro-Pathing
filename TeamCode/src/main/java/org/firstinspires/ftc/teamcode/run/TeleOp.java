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

    Follower follower;
    ArrayList<ButtonLogic> allButtons;

    @Override
    public void Init() {
        /// init stuff here


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
        double axial;
        double lateral;
        double yaw;

        axial   = gamepad1.left_stick_y;
        lateral = gamepad1.left_stick_x;
        yaw     = gamepad1.right_stick_x;

        follower.setTeleOpDrive(axial, lateral, yaw * 0.65, true);

        follower.update();

        updateTelemetry();

    }

    @Override
    public void Stop() {
        follower = null;
    }

    public void updateTelemetry(){
        telemetry.addData("Pose",follower.getPose());
        telemetry.update();
    }
}
