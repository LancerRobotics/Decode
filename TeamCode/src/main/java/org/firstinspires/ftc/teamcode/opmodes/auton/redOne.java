package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="redOne")
public class redOne extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-54,54,Math.toRadians(315)));
        // initial position

        Movement movement = new Movement(hardwareMap);

        waitForStart(); // Required for all autons and teleops


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-54, 54,Math.toRadians(315)))
                        .stopAndAdd(new OuttakeServo(true, hardwareMap)) // close gate
                        .strafeTo(new Vector2d(-24, 24))
                        .turnTo(Math.toRadians(135))

                        .stopAndAdd(new OuttakeMotor(true,hardwareMap))
                        .waitSeconds(3)

                        // Launch Start
                        .stopAndAdd(new Intake(true, hardwareMap)) // start intake
                        .stopAndAdd(new OuttakeMotor(true, hardwareMap)) // start outtake
                        .waitSeconds(1.5)
                        .stopAndAdd(new OuttakeServo(false, hardwareMap)) // open gate
                        .waitSeconds(1)
                        // Launch End
                        // Launch Start
                        .stopAndAdd(new OuttakeServo(true, hardwareMap)) // close gate
                        .stopAndAdd(new Intake(true, hardwareMap)) // start intake
                        .stopAndAdd(new OuttakeMotor(true, hardwareMap)) // start outtake
                        .waitSeconds(1.5)
                        .stopAndAdd(new OuttakeServo(false, hardwareMap)) // open gate
                        .waitSeconds(1)
                        // Launch End
                        // Launch Start
                        .stopAndAdd(new OuttakeServo(true, hardwareMap)) // close gate
                        .stopAndAdd(new Intake(true, hardwareMap)) // start intake
                        .stopAndAdd(new OuttakeMotor(true, hardwareMap)) // start outtake
                        .waitSeconds(1.5)
                        .stopAndAdd(new OuttakeServo(false, hardwareMap)) // open gate
                        .waitSeconds(1)
                        // Launch End
                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(-12,24))
                        .strafeTo(new Vector2d(-12,52))
                        .strafeTo(new Vector2d(-24, 24))
                        .turnTo(Math.toRadians(135))


                        .waitSeconds(1)

                        // Launch Start
                        .stopAndAdd(new OuttakeServo(true, hardwareMap)) // close gate
                        .stopAndAdd(new Intake(true, hardwareMap)) // start intake
                        .stopAndAdd(new OuttakeMotor(true, hardwareMap)) // start outtake
                        .waitSeconds(1.5)
                        .stopAndAdd(new OuttakeServo(false, hardwareMap)) // open gate
                        .waitSeconds(1)
                        // Launch End
                        // Launch Start
                        .stopAndAdd(new OuttakeServo(true, hardwareMap)) // close gate
                        .stopAndAdd(new Intake(true, hardwareMap)) // start intake
                        .stopAndAdd(new OuttakeMotor(true, hardwareMap)) // start outtake
                        .waitSeconds(1.5)
                        .stopAndAdd(new OuttakeServo(false, hardwareMap)) // open gate
                        .waitSeconds(1)
                        // Launch End
                        // Launch Start
                        .stopAndAdd(new OuttakeServo(true, hardwareMap)) // close gate
                        .stopAndAdd(new Intake(true, hardwareMap)) // start intake
                        .stopAndAdd(new OuttakeMotor(true, hardwareMap)) // start outtake
                        .waitSeconds(1.5)
                        .stopAndAdd(new OuttakeServo(false, hardwareMap)) // open gate
                        .waitSeconds(1)
                        // Launch End

                        //parking
                        .strafeTo(new Vector2d(24,0))
                        .turnTo(Math.toRadians(90))


                        .stopAndAdd(new OuttakeMotor(false,hardwareMap))

                        .build()
        );

    }
}
