package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LancersRobot;
import org.firstinspires.ftc.teamcode.LancersTeleOpController;

@TeleOp(name = "LancersTeleOp | Red")
public class LancersTeleOpRed extends LinearOpMode {
    private boolean started = false;


    @Override
    public void runOpMode() throws InterruptedException {

        // Create robot once (hardwareMap + telemetry + gamepads)
        LancersRobot robot = new LancersRobot(hardwareMap, telemetry, false, true, false);
        // outtake speed is currently based off velocity PIDFs, not raw motor power

        LancersTeleOpController controller = new LancersTeleOpController(hardwareMap, robot);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (!started){
                started = true;
                robot.odo.setPosX(120, DistanceUnit.INCH);
                robot.odo.setPosY(24, DistanceUnit.INCH);
                robot.odo.setHeading(90, AngleUnit.DEGREES);
            }
            controller.loop(gamepad1, gamepad2);
        }
    }
}
