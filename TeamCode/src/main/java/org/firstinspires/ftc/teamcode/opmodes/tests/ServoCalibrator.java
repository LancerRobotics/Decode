package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.LancersBotConfig;

@TeleOp(name="ServoCalibrator", group="tests")
public class ServoCalibrator extends LinearOpMode {
    private double intakePower = 0.0;
    private double outtakeVelocity = 0.0;
    private double feederPower = 0.0;

    private double lastOuttakeVelocity = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        final Servo servo = hardwareMap.servo.get(LancersBotConfig.OUTTAKE_SERVO);

        final DcMotor intake = hardwareMap.dcMotor.get(LancersBotConfig.INTAKE_MOTOR);
        final DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, LancersBotConfig.OUTTAKE_MOTOR);
        final DcMotor feeder = hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR_TWO);

        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.REVERSE);

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //PIDFCoefficients otherPIDF = new PIDFCoefficients(120, 1, 10, 20.5);
        //outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, otherPIDF);

        double pos = 0;
        servo.setPosition(pos);

        waitForStart();

        while (opModeIsActive()) {

            // SERVO

            if (gamepad1.dpadUpWasPressed()) {
                pos += 0.1;
            } else if (gamepad1.dpadDownWasPressed()) {
                pos -= 0.1;
            } else if (gamepad1.dpadLeftWasPressed()) {
                pos -= 0.01;
            } else if (gamepad1.dpadRightWasPressed()) {
                pos += 0.01;
            }

            if (pos < 0) pos = 0;
            else if (pos > 1) pos = 1;

            servo.setPosition(pos);

            // INTAKE AND OUTTAKE (if needed)

            if (gamepad1.leftBumperWasPressed()) {
                intakePower = (intakePower == 0) ? 0.85 : 0;
            }
            if (gamepad1.rightBumperWasPressed()) {
                outtakeVelocity = (outtakeVelocity == 0) ? 1260 : 0;
            }
            if (gamepad1.yWasPressed()) {
                feederPower = (feederPower == 0) ? 0.9 : 0;
                lastOuttakeVelocity = (feederPower == 0) ? lastOuttakeVelocity : outtake.getVelocity();
            }

            intake.setPower(intakePower);
            outtake.setVelocity(outtakeVelocity);
            feeder.setPower(feederPower);

            // TELEMETRY

            telemetry.addLine("Use the Dpad buttons to change the position");
            telemetry.addLine("up/down => +0.1/-0.1");
            telemetry.addLine("left/right => +0.01/-0.01");
            telemetry.addData("Servo position", pos);

            telemetry.addData("intake power ", intakePower);
            telemetry.addData("feeder power ", feederPower);
            telemetry.addData("Targeted outtake ", outtakeVelocity);
            telemetry.addData("outtake velocity ", outtake.getVelocity());

            telemetry.addLine("-------------------------");
            telemetry.addData("Last outtake velocity: ", lastOuttakeVelocity);

            telemetry.update();
        }
    }
}
