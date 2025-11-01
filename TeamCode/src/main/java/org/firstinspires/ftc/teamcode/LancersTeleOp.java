package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// hi
@TeleOp()
public class LancersTeleOp extends LinearOpMode {
    public static final String TAG = "LancerTeleOp";
    private long currentRunTimeStamp = -1;
    private long timeStampAtLastOpModeRun = -1;

    private double servoPosition = 0.0;

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

        //intake
        final DcMotor intakeMotor = hardwareMap.dcMotor.get(LancersBotConfig.INTAKE_MOTOR);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        //outtake
        final DcMotor outtakeMotor = hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);

        final Servo outtakeServo = hardwareMap.servo.get(LancersBotConfig.OUTTAKE_SERVO);

        servoPosition = 1;
        outtakeServo.setPosition(1);


        //DcMotorEx inherits from DcMotor class,
        //DcMotorEx used in pretty much the same way as DcMotor
        //I honestly don't really know the difference, but DcMotorEx seems to have more functionality

        //Declaring odometry pods/dead wheels/encoders/whatever you want to call it
        //Pulling from hardware map again
        //parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_RIGHT_MOTOR));
        //perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_LEFT_MOTOR));

        // go!!
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            currentRunTimeStamp = System.currentTimeMillis();

            // movement
            final double speedMultiplier = gamepad1.a ? 1.0d : 0.8d;


            // Gamepad positions; Motors are swapped
            final double ly = -respectDeadZones(gamepad1.left_stick_y) * speedMultiplier; // Remember, Y stick value is reversed
            final double lx = respectDeadZones(gamepad1.left_stick_x) * speedMultiplier; // Counteract imperfect strafing
            final double rx = respectDeadZones(gamepad1.right_stick_x) * speedMultiplier;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            final double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

            //Calculations, more in depth here:
            //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

            final double frontLeftPower = (ly + lx + rx) / denominator;
            final double backLeftPower = (ly - lx + rx) / denominator;
            final double frontRightPower = (ly - lx - rx) / denominator;
            final double backRightPower = (ly + lx - rx) / denominator;

            final double intakePower = (gamepad2.left_trigger>0)?1:0;
            final double outtakePower = (gamepad2.right_trigger>0)?1:0;

            if (gamepad2.y && servoPosition==1){
                servoPosition=0.8;
                outtakeServo.setPosition(0.8);
                sleep(700);
                servoPosition=1;
                outtakeServo.setPosition(1);
            }

            //Speed multipliers by .9, reduces speed of motor
            //Motors get very funky when running at maximum capacity, cap their speed
            leftFront.setPower(-frontLeftPower);
            leftRear.setPower(-backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            intakeMotor.setPower(intakePower);
            outtakeMotor.setPower(outtakePower);
            // we finished an iteration, record the time the last value was recorded for use in finding sum
            timeStampAtLastOpModeRun = currentRunTimeStamp;

            //telemetry.addData("X-value", parallelEncoder.getCurrentPosition());
            //telemetry.addData("Y-value", perpendicularEncoder.getCurrentPosition());

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