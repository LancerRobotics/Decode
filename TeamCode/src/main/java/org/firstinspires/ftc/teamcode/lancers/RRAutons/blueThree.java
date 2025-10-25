package org.firstinspires.ftc.teamcode.lancers.RRAutons;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lancers.MecanumDrive;

@Autonomous(name="blueThree")
public class blueThree extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12,132,Math.toRadians(45)));
        // initial position

        Movement movement = new Movement(hardwareMap);

        waitForStart(); // Required for all autons and teleops


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(24, 120,Math.toRadians(45)))
                        .strafeTo(new Vector2d(48, 96))
                        .turnTo(Math.toRadians(135))
                        .stopAndAdd(new OuttakeMotor(true))
                        .stopAndAdd(new OuttakeServo(1))

                        .build()
        );

    }
}
