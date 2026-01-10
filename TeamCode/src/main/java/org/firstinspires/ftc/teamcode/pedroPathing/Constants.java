package org.firstinspires.ftc.teamcode.pedroPathing;

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
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.02, 0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.03, 0.015))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.1, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0.015, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.25, 0, 0.00001, 0.6, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01));
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("Motor Front Right") // change later
            .rightRearMotorName("Motor Back Right") // change later
            .leftRearMotorName("Motor Back Left") // change later
            .leftFrontMotorName("Motor Front Left") // change later
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(63)
            .yVelocity(47);
    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.00787402)
            .strafePodX(-7.28346457)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("Pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}