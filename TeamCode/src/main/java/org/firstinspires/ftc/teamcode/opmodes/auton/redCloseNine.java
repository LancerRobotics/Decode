package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name="redCloseNine")
public class redCloseNine extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-52, 51,Math.toRadians(225)));
        // initial position



        Movement movement = new Movement(hardwareMap);

        Vector2d launchPoint = new Vector2d(-12,12);

        waitForStart(); // Required for all autons and teleops


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-52, 51,Math.toRadians(315)))

                        // Strafe to shooting position
                        .stopAndAdd(new Intake(true, hardwareMap))
                        .strafeTo(launchPoint)

                        // Shoot 3
                        .stopAndAdd(new OuttakeMotor(true,hardwareMap))
                        .waitSeconds(0.5)
                        .stopAndAdd(new OuttakeMotorTwo(true,hardwareMap))
                        .waitSeconds(3)
                        .stopAndAdd(new OuttakeMotor(false,hardwareMap))
                        .stopAndAdd(new OuttakeMotorTwo(false,hardwareMap))

                        // Go to first ball area and intake 3
                        .turnTo(Math.toRadians(-270))
                        .strafeTo(new Vector2d(-8,24))
                        .strafeTo(new Vector2d(-8,58))

                       // Strafe to shooting position
                        .strafeTo(launchPoint)
                        .turnTo(Math.toRadians(315))

                        // Shoot 3
                        .stopAndAdd(new OuttakeMotor(true,hardwareMap))
                        .waitSeconds(0.5)
                        .stopAndAdd(new OuttakeMotorTwo(true,hardwareMap))
                        .waitSeconds(3)
                        .stopAndAdd(new OuttakeMotor(false,hardwareMap))
                        .stopAndAdd(new OuttakeMotorTwo(false,hardwareMap))

                        // Go to second ball area and intake 3
                        .turnTo(Math.toRadians(-270))
                        .strafeTo(new Vector2d(16,24))
                        .strafeTo(new Vector2d(16,58))

                        // Strafe to shooting position
                        .strafeTo(launchPoint)
                        .turnTo(Math.toRadians(315))

                        // Shoot 3
                        .stopAndAdd(new OuttakeMotor(true,hardwareMap))
                        .waitSeconds(0.5)
                        .stopAndAdd(new OuttakeMotorTwo(true,hardwareMap))
                        .waitSeconds(3)
                        .stopAndAdd(new OuttakeMotor(false,hardwareMap))
                        .stopAndAdd(new OuttakeMotorTwo(false,hardwareMap))

                        // Strafe to final position and end auton
                        .strafeTo(new Vector2d(-24,50))
                        //.turnTo(Math.toRadians(-90))
                        .stopAndAdd(new OuttakeMotor(false,hardwareMap))
                        .stopAndAdd(new OuttakeMotorTwo(false,hardwareMap))

                        .build()
        );

    }
}
