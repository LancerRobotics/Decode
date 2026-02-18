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
import org.firstinspires.ftc.teamcode.LancersBotConfig;

// use 192.168.43.1:8001 in order to connect to PanelsTelemetry

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.52)
            .forwardZeroPowerAcceleration(-38.092965) //-36.228636, -37.375510
            .lateralZeroPowerAcceleration(-72.130466) //-63.620337, -76.621186
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0.072))
            .headingPIDFCoefficients(new PIDFCoefficients(0.37,0,0.015,0.05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0,0.0007,0.6,0.046))
            .centripetalScaling(0.0008);


    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.2,
            0.85); // TODO: This needs to be tuned

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(LancersBotConfig.FRONT_RIGHT_MOTOR)
            .rightRearMotorName(LancersBotConfig.REAR_RIGHT_MOTOR)
            .leftRearMotorName(LancersBotConfig.REAR_LEFT_MOTOR)
            .leftFrontMotorName(LancersBotConfig.FRONT_LEFT_MOTOR)

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77.485806)
            .yVelocity(61.586316);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.5625)
            .strafePodX(-5.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(LancersBotConfig.PINPOINT)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}