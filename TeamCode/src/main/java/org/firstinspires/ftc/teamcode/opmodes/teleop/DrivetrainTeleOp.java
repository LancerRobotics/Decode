package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LancersBotConfig;

// hi
@TeleOp()
public class DrivetrainTeleOp extends LinearOpMode {
    public static final String TAG = "Drivetrain TeleOp";
    private long currentRunTimeStamp = -1;
    private long timeStampAtLastOpModeRun = -1;

    private boolean isDriving;
    private boolean leftBumperLastPressed;

    @Override
    public void runOpMode() throws InterruptedException  {
        // Get from hardwaremap, initialize variables as DcMotor type
        final DcMotor leftFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotorEx leftRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotor rightFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotor rightRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        isDriving = true;

        // go!!
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

            if (isDriving) { // Driving bot, default
                // movement
                final double speedMultiplier = gamepad1.a ? 1.0d : 0.8d;


                // Gamepad positions; Motors are swapped
                final double ly = -respectDeadZones(gamepad1.left_stick_y) * speedMultiplier; // Remember, Y stick value is reversed
                final double lx = respectDeadZones(gamepad1.left_stick_x) * speedMultiplier;
                final double rx = respectDeadZones(gamepad1.right_stick_x) * speedMultiplier;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                final double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

                //Calculations, more in depth here:
                //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

                frontLeftPower = (ly + lx + rx) / denominator;
                backLeftPower = (ly - lx + rx) / denominator;
                frontRightPower = (ly - lx - rx) / denominator;
                backRightPower = (ly + lx - rx) / denominator;
            }
            else { // motor debugging


                frontLeftPower = gamepad1.x ? 1 : 0;
                backLeftPower = gamepad1.a ? 1 : 0;
                frontRightPower = gamepad1.y ? 1 : 0;
                backRightPower = gamepad1.b ? 1 : 0;
            }


            //Speed multipliers by .9, reduces speed of motor
            //Motors get very funky when running at maximum capacity, cap their speed
            leftFront.setPower(-frontLeftPower);
            leftRear.setPower(-backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            if (gamepad1.left_bumper && !leftBumperLastPressed) {
                isDriving = !isDriving;
            }
            leftBumperLastPressed = gamepad1.left_bumper;

            // Telemetry
            telemetry.addLine("Press on the Left Bumper to Switch Modes");

            telemetry.addLine("\nFor Motor Debugging, rotate your controller 45 degrees clockwise.");
            telemetry.addLine("The positions of the buttons indicate the wheel associated with it.");
            telemetry.addLine("For example, the Front Left Motor is associated with the Front Left Button which is X.");

            telemetry.addLine("\nCurrent mode: "+(isDriving ? "Driving" : "Motor Debugging")); // Driving / motor debugging
            telemetry.addData("X-value", leftFront.getCurrentPosition());
            telemetry.addData("Y-value", rightFront.getCurrentPosition());


            telemetry.update();
        }
    }

    public static final double DEAD_ZONE_LIMIT = 0.15d;

    /**
     * Stick return is unreliable near inside, toss signals that are less than a threshold to maintain stationary behaviour when sticks may or may not be being minimally actuated by using this method to wrap a double value.
     * @param input
     * @return
     */
    public static double respectDeadZones(double input) {
        if (Math.abs(input) < DEAD_ZONE_LIMIT) {
            return 0.0d;
        } else {
            return input;
        }
    }
}