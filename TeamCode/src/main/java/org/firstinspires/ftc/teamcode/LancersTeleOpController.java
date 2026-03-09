package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private double outtakePower = 0.0; // 0 or 1
    private double outtakeTwoPower = 0.0; // 0.0 or 0.5

    private boolean outtakeTwoReversed = true; // starts reversed

    // Servo state: 0.0 or 0.5 (your “open/closed” positions)
    private double servoPos = 0;
    private LimelightWrapper limelightWrapper;

    private LancersRobot robot;

    private final ElapsedTime calibrationTimer = new ElapsedTime();
    private double lastCalibrationTime = 0;

    public LancersTeleOpController(HardwareMap hardwareMap, LancersRobot robot) {
        limelightWrapper = new LimelightWrapper(robot.getHardwareMap());
        this.robot = robot;
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
/*
        // Recalibrate Pinpoint odometry from vision every 500 ms when a tag is visible
        if (calibrationTimer.milliseconds() - lastCalibrationTime > 5000) {
            limelightWrapper.calibratePinpoint(robot.odo, robot.getCurrentTurretDeg());
            lastCalibrationTime = calibrationTimer.milliseconds();
        }
*/
        /*
        // Turret "locking" mechanism
        boolean tagVisible = limelightWrapper.tagSeen();
        double turretJoystick = gamepad2.right_stick_x;

        if (tagVisible && limelightWrapper.getTagId() == 20) {
            // This is the "aimbot" method where the limelight tracks the tag (ever so slightly off)
            robot.aimTurretToAngle(robot.getIntegratedAngle(false), 0.8,1);
        }
        else if (tagVisible && limelightWrapper.getTagId() == 24) {
            robot.aimTurretToAngle(robot.getIntegratedAngle(true),0.8,1);
        }
        else if (Math.abs(turretJoystick) > 0.1) {
            robot.setOuttakeRotationMotor(turretJoystick);
        } else {
            robot.setOuttakeRotationMotor(0);
        }


        if (gamepad1.dpadLeftWasPressed()){
            robot.changeTurretOffset(-10);
        }
        if (gamepad1.dpadRightWasPressed()){
            robot.changeTurretOffset(10);
        }

         */

        // DRIVETRAIN

        final double multiplier = gamepad1.a ? 1.0 : 0.8;

        final double ly = -LancersRobot.respectDeadZones(gamepad1.left_stick_y);
        final double lx =  LancersRobot.respectDeadZones(gamepad1.left_stick_x);
        final double rx =  LancersRobot.respectDeadZones(gamepad1.right_stick_x);

        robot.driveMecanum(ly, lx, rx, multiplier);


        // INTAKE
        if (gamepad1.leftBumperWasPressed()) {
            //intakeOn = (intakeOn == 0.0) ? 1.0 : 0.0;
            intakeOn = (intakeOn == 0.0) ? 0.85 : 0.0;
        }

        if (gamepad1.rightBumperWasPressed()) {
            intakeDirection *= -1.0;
        }

        if(gamepad2.bWasPressed()) {
            robot.resetOuttakeRotationMotorPosition();
        }

        robot.setIntake(intakeOn * intakeDirection);



        // OUTTAKE


        if (gamepad2.rightBumperWasPressed()) {
        //if (gamepad2.rightBumperWasPressed()) {
            //outtakeVel = (outtakeVel == 0.0) ? 1260 : 0.0;
            //outtakePower = (outtakePower == 0.0) ? 1:0;
            robot.setAdjustedOuttakeVelocity();
        }

        if (gamepad2.leftBumperWasPressed()) {
            if (outtakeTwoPower == 0) {
                outtakeTwoPower = outtakeTwoReversed ? -0.9 : 0.9;
            } else {
                outtakeTwoReversed = !outtakeTwoReversed;
                outtakeTwoPower = outtakeTwoReversed ? -0.9 : 0.9;
            }
        }

        robot.setOuttakeTwoPower(outtakeTwoPower);



        //robot.setOuttakeVelocity(outtakeVel);
        //robot.setOuttakePower(outtakePower);
        robot.autoAdjustOuttakeVelocity();

        robot.setOuttakeTwoPower(outtakeTwoPower);

        if (gamepad1.yWasPressed()) {
            servoPos = (servoPos == 0) ? 1 : 0;
            robot.setServoPosition(servoPos);
        }

        if (gamepad2.aWasPressed()) {robot.setOriginalTime();}

        if (gamepad1.xWasPressed()) {robot.relocalizeRobot();}

        //if (gamepad2.rightBumperWasPressed()) {robot.setTurretMode();}

        //robot.aimReset();
        //robot.aimOuttakeToTx(0.5);

        // After the gamepad controls are in, update the robot and send telemetry
        robot.update();
        robot.sendTelemetry(true);
    }
}