package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LancersRobot;
import org.firstinspires.ftc.teamcode.LancersTeleOpController;

@TeleOp()
public class LancersTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Create robot once (hardwareMap + telemetry + gamepads)
        LancersRobot robot = new LancersRobot(hardwareMap, telemetry, true);
        // outtake speed is currently based off velocity PIDFs, not raw motor power

        robot.odo.resetPosAndIMU();

        LancersTeleOpController controller = new LancersTeleOpController();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            controller.loop(robot, gamepad1, gamepad2);
        }
    }
}
