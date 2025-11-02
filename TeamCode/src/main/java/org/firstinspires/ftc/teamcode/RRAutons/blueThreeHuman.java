package org.firstinspires.ftc.teamcode.RRAutons;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="blueThreeHuman")
public class blueThreeHuman extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-54, -54,Math.toRadians(45)));
        // initial position

        Movement movement = new Movement(hardwareMap);

        waitForStart(); // Required for all autons and teleops


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-54, -54,Math.toRadians(45)))
                        .stopAndAdd(new OuttakeMotor(true,hardwareMap))
                        .strafeTo(new Vector2d(-24, -24))
                        .turnTo(Math.toRadians(230))


                        .waitSeconds(0.3)

                        // Launch Start
                        .stopAndAdd(new OuttakeServo(0.8,hardwareMap))
                        .waitSeconds(0.7)
                        .stopAndAdd(new OuttakeServo(1,hardwareMap))
                        .waitSeconds(1)
                        // Launch End
                        // Launch Start
                        .stopAndAdd(new OuttakeServo(0.8,hardwareMap))
                        .waitSeconds(0.7)
                        .stopAndAdd(new OuttakeServo(1,hardwareMap))
                        .waitSeconds(1)
                        // Launch End
                        // Launch Start
                        .stopAndAdd(new OuttakeServo(0.8,hardwareMap))
                        .waitSeconds(0.7)
                        .stopAndAdd(new OuttakeServo(1,hardwareMap))
                        //.waitSeconds(1)
                        // Launch End

                        .turnTo(Math.toRadians(45))
                        .strafeTo(new Vector2d(36, 36))
                        .turnTo(Math.toRadians(-90)) // 270 deg
                        .strafeTo(new Vector2d(60, 60))

                        .waitSeconds(4)

                        .strafeTo(new Vector2d(36, 36))
                        .turnTo(Math.toRadians(230))
                        .strafeTo(new Vector2d(-24, -24))

                        // Launch Start
                        .stopAndAdd(new OuttakeServo(0.8,hardwareMap))
                        .waitSeconds(0.7)
                        .stopAndAdd(new OuttakeServo(1,hardwareMap))
                        .waitSeconds(1)
                        // Launch End
                        // Launch Start
                        .stopAndAdd(new OuttakeServo(0.8,hardwareMap))
                        .waitSeconds(0.7)
                        .stopAndAdd(new OuttakeServo(1,hardwareMap))
                        .waitSeconds(1)
                        // Launch End
                        // Launch Start
                        .stopAndAdd(new OuttakeServo(0.8,hardwareMap))
                        .waitSeconds(0.7)
                        .stopAndAdd(new OuttakeServo(1,hardwareMap))
                        .waitSeconds(1)
                        // Launch End

                        .stopAndAdd(new OuttakeMotor(false,hardwareMap))

                        .build()
        );

    }
}
