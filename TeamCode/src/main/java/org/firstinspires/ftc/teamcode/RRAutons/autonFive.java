package org.firstinspires.ftc.teamcode.RRAutons;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="autonFive")
public class autonFive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(56, 12,Math.toRadians(180)));
        // initial position

        Movement movement = new Movement(hardwareMap);

        waitForStart(); // Required for all autons and teleops


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(56, 12,Math.toRadians(180)))
                        .stopAndAdd(new OuttakeMotor(true,hardwareMap))

                        .strafeTo(new Vector2d(56, 36))
                        .stopAndAdd(new OuttakeMotor(false,hardwareMap))

                        .build()
        );

    }
}
