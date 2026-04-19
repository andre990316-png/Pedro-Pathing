package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Config;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15)
            .forwardZeroPowerAcceleration(-38.27)
            .lateralZeroPowerAcceleration(-70.15)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.02, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.04, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0002, 0.6, 0));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(Config.motor_lf.id)
            .leftRearMotorName(Config.motor_lb.id)
            .rightFrontMotorName(Config.motor_rf.id)
            .rightRearMotorName(Config.motor_rb.id)
            .leftFrontMotorDirection(Config.motor_lf.direction)
            .leftRearMotorDirection(Config.motor_lb.direction)
            .rightFrontMotorDirection(Config.motor_rf.direction)
            .rightRearMotorDirection(Config.motor_rb.direction)
            .xVelocity(73.26)   //forward max power
            .yVelocity(52.26)   // <-  max power
            .useBrakeModeInTeleOp(false);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.4375)
            .strafePodX(-5.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(Config.odometry.id)
            .yawScalar(1.0)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(Config.odometry.forward_direction)
            .strafeEncoderDirection(Config.odometry.strafe_direction);
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            50,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
