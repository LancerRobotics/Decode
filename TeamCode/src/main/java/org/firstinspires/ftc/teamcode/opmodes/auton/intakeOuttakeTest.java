package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="intakeOuttakeTest")
public class intakeOuttakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(0)));
        // initial position

        Movement movement = new Movement(hardwareMap);

        waitForStart(); // Required for all autons and teleops

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0,Math.toRadians(0)))
                        .stopAndAdd(new OuttakeServo(true, hardwareMap)) // close gate

                        .stopAndAdd(new Intake(true, hardwareMap)) // start intake
                        .stopAndAdd(new OuttakeMotor(true, hardwareMap)) // start outtake
                        .waitSeconds(1.5)
                        .stopAndAdd(new OuttakeServo(false, hardwareMap)) // open gate
                        .waitSeconds(10)
                        .build()
        );

    }
}
