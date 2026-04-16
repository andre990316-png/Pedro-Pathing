package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.7)
            .forwardZeroPowerAcceleration(-32)
            .lateralZeroPowerAcceleration(-67)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.02, 0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.03, 0.015))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.1, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.015, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.00001, 0.6, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.000005, 0.6, 0.01));
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RF") // change later
            .rightRearMotorName("RB") // change later
            .leftRearMotorName("LB") // change later
            .leftFrontMotorName("LF") // change later
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(63)
            .yVelocity(47);
    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-9.5)
            .strafePodX(-15.5)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}