package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.LancersBotConfig;

// This Opmode is based on this video: https://youtu.be/aPNCpZzCTKg?si=UInPiWX1i2hniq1L

@TeleOp(name="flywheelPIDFTuner", group="tests")
public class flywheelPIDFTuner extends OpMode {

    public DcMotorEx flywheelMotor;
    public DcMotor intakeMotor;
    public DcMotor feederMotor;
    public DcMotorEx outtakeRotationMotor;

    public double highVelocity = 1260;
    public double lowVelocity = 1080;

    private double intakePower = 0.0;
    private double feederPower = 0.0;

    double curTargetVelocity = highVelocity;

    // This Opmode is only tuning P and F
    double P = 0; // 59.6, 50.04
    final double I = 0;
    final double D = 0;
    double F = 0; // 19.8, 20.84

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, LancersBotConfig.OUTTAKE_MOTOR);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");

        intakeMotor = hardwareMap.dcMotor.get(LancersBotConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        feederMotor = hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR_TWO);

        outtakeRotationMotor = hardwareMap.get(DcMotorEx.class, LancersBotConfig.OUTTAKE_ROTATION_MOTOR);
        outtakeRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRotationMotor.setPower(0);
    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.leftBumperWasPressed()) {
            intakePower = (intakePower == 0) ? 0.85 : 0;
        }
        if (gamepad1.rightBumperWasPressed()) {
            feederPower = (feederPower == 0) ? 0.9 : 0;
        }

        intakeMotor.setPower(intakePower);
        feederMotor.setPower(feederPower);

        //set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("---------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
