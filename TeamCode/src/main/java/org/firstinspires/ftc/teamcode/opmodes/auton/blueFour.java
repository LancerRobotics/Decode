package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="blueFour")
public class blueFour extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(60, -12,Math.toRadians(188.1301024)));
        // initial position

        Movement movement = new Movement(hardwareMap);

        waitForStart(); // Required for all autons and teleops


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(60, -12,Math.toRadians(180)))
                        .stopAndAdd(new OuttakeMotor(true,hardwareMap))

                        .strafeTo(new Vector2d(-24, -27))
                        .turnTo(Math.toRadians(255))

                        .waitSeconds(1)

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
                        .turnTo(Math.toRadians(270))
                        .strafeTo(new Vector2d(-12,-24))
                        .strafeTo(new Vector2d(-12,-52))
                        .strafeTo(new Vector2d(-24, -24))
                        .turnTo(Math.toRadians(230))


                        .waitSeconds(1)

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

                        .strafeTo(new Vector2d(24,0))
                        .turnTo(Math.toRadians(90))

                        .stopAndAdd(new OuttakeMotor(false,hardwareMap))

                        .build()
        );

    }
}
