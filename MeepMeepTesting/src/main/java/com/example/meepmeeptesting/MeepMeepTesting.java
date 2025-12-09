package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Vector2d launchPoint = new Vector2d(-24,24);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-52, 51,Math.toRadians(135))) // Strafe to shooting position

                // Strafe to shooting position
                //.stopAndAdd(new Intake(true, hardwareMap))
                .strafeTo(launchPoint)

                // Shoot 3
               // .stopAndAdd(new OuttakeMotor(true,hardwareMap))
                .waitSeconds(0.5)
               // .stopAndAdd(new OuttakeMotorTwo(true,hardwareMap))
                .waitSeconds(3)
                //.stopAndAdd(new OuttakeMotor(false,hardwareMap))
                //.stopAndAdd(new OuttakeMotorTwo(false,hardwareMap))

                // Go to first ball area and intake 3
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-9,24))
                .strafeTo(new Vector2d(-9,58))

                // Strafe to shooting position
                .strafeTo(launchPoint)
                .turnTo(Math.toRadians(135))

                // Shoot 3
                //.stopAndAdd(new OuttakeMotor(true,hardwareMap))
                .waitSeconds(0.5)
                //.stopAndAdd(new OuttakeMotorTwo(true,hardwareMap))
                .waitSeconds(3)
                //.stopAndAdd(new OuttakeMotor(false,hardwareMap))
               // .stopAndAdd(new OuttakeMotorTwo(false,hardwareMap))

                // Go to second ball area and intake 3
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(15,24))
                .strafeTo(new Vector2d(15,58))

                // Strafe to shooting position
                .strafeTo(launchPoint)
                .turnTo(Math.toRadians(135))

                // Shoot 3
                //.stopAndAdd(new OuttakeMotor(true,hardwareMap))
                .waitSeconds(0.5)
                //.stopAndAdd(new OuttakeMotorTwo(true,hardwareMap))
                .waitSeconds(3)
                //.stopAndAdd(new OuttakeMotor(false,hardwareMap))
                //.stopAndAdd(new OuttakeMotorTwo(false,hardwareMap))

                // Strafe to final position and end auton
                .strafeTo(new Vector2d(-24,50))
                //.turnTo(Math.toRadians(-90))
                //.stopAndAdd(new OuttakeMotor(false,hardwareMap))
                //.stopAndAdd(new OuttakeMotorTwo(false,hardwareMap))

                .build()

        );
            System.out.println("done");


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}