package org.firstinspires.ftc.teamcode.OpModeTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.LancersBotConfig;

@TeleOp()
public class ServoCalibrator extends LinearOpMode {
    public static final String TAG = "ServoCalibrator";

    @Override
    public void runOpMode() throws InterruptedException {
        final Servo servo = hardwareMap.servo.get(LancersBotConfig.OUTTAKE_SERVO);
        double pos = 0;
        servo.setPosition(pos);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                pos += 0.1;
                servo.setPosition(pos);
            } else if (gamepad1.a) {
                pos -= 0.1;
                servo.setPosition(pos);
            } else if (gamepad1.x) {
                pos -= 0.01;
                servo.setPosition(pos);
            } else if (gamepad1.b) {
                pos += 0.01;
                servo.setPosition(pos);
            }

            if (pos < 0) pos = 0;
            else if (pos > 1) pos = 1;

            telemetry.addData("Press Y to increase position 0.1", "Press A to decrease position 0.1");
            telemetry.addData("Press X to decrease position 0.01", "Press B to increase position 0.01");
            telemetry.addData("Servo current position", pos);
            telemetry.update();
        }
    }
}
