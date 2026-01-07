package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LancersRobot;

@TeleOp()
public class LancersTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Create robot once (hardwareMap + telemetry + gamepads)
        LancersRobot robot = new LancersRobot(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.loop();
        }
    }
}
