package org.firstinspires.ftc.teamcode.RRAutons;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="outtakeTest")
public class outtakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(0)));
        // initial position

        Movement movement = new Movement(hardwareMap);

        waitForStart(); // Required for all autons and teleops

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0,Math.toRadians(0)))
                        .stopAndAdd(new OuttakeMotor(true, hardwareMap)) // on
                        .waitSeconds(3)
                        .stopAndAdd(new OuttakeServo(0.3,hardwareMap))
                        .waitSeconds(2)
                        .stopAndAdd(new OuttakeServo(0.7,hardwareMap))
                        .waitSeconds(2)
                        .stopAndAdd(new OuttakeMotor(false, hardwareMap)) // off
                        .build()
        );

    }
}
