package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.ClosedLoopControlCoefficients;
import org.firstinspires.ftc.teamcode.vision.LimelightWrapper;

/**
 * Controls all controller logic and send it to LancersRobot
 */
public class LancersTeleOpController {


    // Intake
    private double intakeDirection = 1.0; // +1 or -1
    private double intakeOn = 0.0; // 0 or 1

    // Outtake state: velocity toggle + second motor power toggle
    private double outtakeVel = 0.0; // 0.0 or 1500.0 (ticks/sec, depending on your motor/SDK)\
    private double outtakeTwoPower = 0.0; // 0.0 or 0.5

    // Servo state: 0.0 or 0.5 (your “open/closed” positions)
    private double servoPos = 1.0;
    private LancersRobot robot;
    private LimelightWrapper limelightWrapper;

    public void loop(LancersRobot robot, Gamepad gamepad1, Gamepad gamepad2) {

        // Turret "locking" mechanism
        boolean tagVisible = limelightWrapper.tagSeen();
        double turretJoystick = gamepad2.right_stick_x;

        if (tagVisible) {
            // This is the "aimbot" method where the limelight tracks the tag (ever so slightly off)
            robot.aimOuttakeToTx(5);
        } else if (Math.abs(turretJoystick) > 0.1) {
            robot.setOuttakeRotationMotor(turretJoystick);
        } else {
            robot.setOuttakeRotationMotor(0);
        }

        // DRIVETRAIN

        final double multiplier = gamepad1.a ? 1.0 : 0.8;

        final double ly = -LancersRobot.respectDeadZones(gamepad1.left_stick_y);
        final double lx =  LancersRobot.respectDeadZones(gamepad1.left_stick_x);
        final double rx =  LancersRobot.respectDeadZones(gamepad1.right_stick_x);

        robot.driveMecanum(ly, lx, rx, multiplier);


        // INTAKE
        if (gamepad1.leftBumperWasPressed()) {
            intakeOn = (intakeOn == 0.0) ? 1.0 : 0.0;
        }

        if (gamepad1.rightBumperWasPressed()) {
            intakeDirection *= -1.0;
        }

        robot.setIntake(intakeOn * intakeDirection);

        // OUTTAKE
        if (gamepad2.rightBumperWasPressed()) {
            outtakeVel = (outtakeVel == 0.0) ? 1500.0 : 0.0;
        }

        if (gamepad2.leftBumperWasPressed()) {
            outtakeTwoPower = (outtakeTwoPower == 0.0) ? 1 : 0.0;
        }

        robot.setOuttakeVelocity(outtakeVel);
        robot.setOuttakeTwoPower(outtakeTwoPower);

        if (gamepad2.yWasPressed()) {
            servoPos = (servoPos == 1.0) ? 0.4 : 1.0;
            robot.setServoPosition(servoPos);
        }

        if (gamepad2.aWasPressed()) {robot.setOriginalTime();}

        robot.aimReset();
        robot.aimOuttakeToTx(0.5);

        // After the gamepad controls are in, update the robot and send telemetry
        robot.update();
        robot.sendTelemetry(true);
    }
}