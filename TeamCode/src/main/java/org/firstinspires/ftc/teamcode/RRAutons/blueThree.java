package org.firstinspires.ftc.teamcode.RRAutons;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="blueThree")
public class blueThree extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-54,54,Math.toRadians(45)));
        // initial position

        Movement movement = new Movement(hardwareMap);

        waitForStart(); // Required for all autons and teleops


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-54, 54,Math.toRadians(45)))
                        .strafeTo(new Vector2d(0, 24))
                        .turnTo(Math.toRadians(-45))

                        .stopAndAdd(new OuttakeMotor(true, hardwareMap))
                        .stopAndAdd(new OuttakeServo(0.7,hardwareMap))
                        .waitSeconds(3)
                        .stopAndAdd(new OuttakeServo(0,hardwareMap))




                        .build()
        );

    }
}
