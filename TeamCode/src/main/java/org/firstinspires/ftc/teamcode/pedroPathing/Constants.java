package org.firstinspires.ftc.teamcode.pedroPathing;

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
            .mass(10.12)
            .forwardZeroPowerAcceleration(-30.46840712724944);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

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
            .xVelocity(57.006134273499015)
            .yVelocity(32.32423388864112);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.5625)
            .strafePodX(7.8125)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(LancersBotConfig.PINPOINT)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}