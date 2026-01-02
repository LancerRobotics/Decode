package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LancersBotConfig;


// hi
@TeleOp()
public class LancersTeleOp extends LinearOpMode {

    private Limelight3A limelight;

    public static final String TAG = "LancerTeleOp";
    private long currentRunTimeStamp = -1;
    private long timeStampAtLastOpModeRun = -1;

    private double servoPosition = 0.0;

    // DRIVETRAIN SPEED MULTIPLIER LOCK VARIABLES
    private boolean fullSpeedLock;
    private boolean lastLock;


    // INTAKE VARIABLES
    private double intakeDirection = 1; // +1 or -1
    private double intakePower = 0; // 0 or 1
    private double outtakePower = 0;
    private double outtakeTwoPower = 0; // 0 or 1
    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastY = false;
    private boolean lastServo = false;
    private boolean lastOuttakeTwo = false;
    private boolean lastOuttake = false;
    private double outtakeMotorLine = 0;


    @Override
    public void runOpMode() throws InterruptedException  {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50); // around 20 hZ



        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }
        }
        // Get from hardwaremap, initialize variables as DcMotor type
        final DcMotorEx leftFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotorEx leftRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotorEx rightFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotorEx rightRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        //intake
        final DcMotor intakeMotor = hardwareMap.dcMotor.get(LancersBotConfig.INTAKE_MOTOR);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        //outtake
        final DcMotorEx outtakeMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotorEx outtakeMotorTwo = (DcMotorEx)hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR_TWO);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);

        final double TICKS_PER_REV = 1534.4; // CHANGE THIS DEPENDING ON THE MOTOR MODEL!!!!

        final Servo outtakeServo = hardwareMap.servo.get(LancersBotConfig.OUTTAKE_SERVO);

        servoPosition = 0;

        outtakeServo.setPosition(0); // servo intital position

        fullSpeedLock = false;
        lastLock = false;

        intakePower = 0;
        outtakePower = 0;
        outtakeTwoPower = 0;
        intakeDirection = 1;


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
            // OUTTAKE MOTOR STUFF (STILL NEED TO BE TESTED)
            double outtakeTicksPerSec = outtakeMotor.getVelocity();
            double outtakeRPM1 = (outtakeTicksPerSec / TICKS_PER_REV) * 60.0;
            double outtakeTicksPerSec2 = outtakeMotorTwo.getVelocity();
            double outtakeRPM2 = (outtakeTicksPerSec2 / TICKS_PER_REV) * 60.0;

            telemetry.addData("Outtake 1 RPM: ", outtakeRPM1);
            telemetry.addData("Outtake 2 RPM: ", outtakeRPM2);

            /*if(outtakeRPM1 >= 90 || outtakeRPM2 >= 90) {
                //vibrate the gamepads
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }*/

            currentRunTimeStamp = System.currentTimeMillis();

            // movement
            final double speedMultiplier = gamepad1.a ? 1.0d : 0.8d;

            // full speed
            if (gamepad1.b && !lastLock) {
                fullSpeedLock = !fullSpeedLock;
            }
            lastLock = gamepad1.b;


            // Gamepad positions; Motors are swapped
            final double ly = -respectDeadZones(gamepad1.left_stick_y) * (fullSpeedLock ? 1.0d : speedMultiplier); // Remember, Y stick value is reversed
            final double lx = respectDeadZones(gamepad1.left_stick_x) * (fullSpeedLock ? 1.0d : speedMultiplier);
            final double rx = respectDeadZones(gamepad1.right_stick_x) * (fullSpeedLock ? 1.0d : speedMultiplier);

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


            boolean left = gamepad1.left_bumper;     // on/off toggle
            boolean right = gamepad1.right_bumper;   // direction toggle

            // INTAKE ON/OFF (left bumper)
            if (left && !lastLeft) {
                if (intakePower == 0){
                    intakePower = 1;
                }
                else {
                    intakePower = 0;
                }
            }

            /*
            //SECOND OUTTAKE MOTOR TOGGLE (y button)
            if (y_button && !lastY) {
                if (outtakeTwoPower == 0){
                  outtakeTwoPower = 1;
                }
                else {
                    outtakeTwoPower = 0;
                }
            }*/

            // INTAKE DIRECTION (right bumper)
            if (right && !lastRight) {
                intakeDirection *= -1;
            }

            lastLeft = left;
            lastRight = right;

            /*if (gamepad2.right_bumper && !lastOuttakeTwo) {
                if (outtakeTwoPower == 0){
                    outtakeTwoPower = -0.5;
                }
                else {
                    outtakeTwoPower = 0;
                }
            }
            lastOuttakeTwo = gamepad2.right_bumper;*/

            double ticksPerSec = outtakeMotor.getVelocity(AngleUnit.DEGREES);

            /*if ((respectDeadZones(gamepad2.right_trigger)>0) && !lastOuttake) {
                outtakePower = 0.9;
            }
            if (!(respectDeadZones(gamepad2.right_trigger)>0)) {
                outtakePower = 0;
            }
            lastOuttake = (respectDeadZones(gamepad2.right_trigger)>0);
            */
            if (gamepad2.rightBumperWasPressed()){
                outtakePower = (outtakePower == 0.95 ? 0 : 0.95);
            }

            if (ticksPerSec>260){
                outtakePower -= 0.001;
            }

            intakeMotor.setPower(intakePower*intakeDirection);
            outtakeMotorTwo.setPower(outtakeTwoPower);
            outtakeMotor.setPower(outtakePower);

            //else if (respectDeadZones(gamepad1.right_trigger) > 0){
            //    intakeMotor.setPower((-gamepad1.right_trigger));
            //}

            // TEMP
            //final double intakePower = (gamepad1.left_trigger>0)?1:0;



            if (gamepad2.left_bumper && !lastServo) {
                if (servoPosition == 0) {
                    servoPosition = 0.5;
                    outtakeServo.setPosition(0.5); // close position, ready to intake
                } else {
                    servoPosition = 0;
                    outtakeServo.setPosition(0); // open position, ready to launch
                }
            }

            lastServo = gamepad2.left_bumper;


            if (gamepad2.yWasPressed()){
                outtakeMotorLine = ticksPerSec;
            }

            //Speed multipliers by .9, reduces speed of motor
            //Motors get very funky when running at maximum capacity, cap their speed
            leftFront.setPower(-frontLeftPower);
            leftRear.setPower(-backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            // TOOD: TEMP, FIX LATER
            //intakeMotor.setPower(intakePower);
            //outtakeMotor.setPower(outtakePower);

            // we finished an iteration, record the time the last value was recorded for use in finding sum
            timeStampAtLastOpModeRun = currentRunTimeStamp;

            //telemetry.addData("X-value", parallelEncoder.getCurrentPosition());
            //telemetry.addData("Y-value", perpendicularEncoder.getCurrentPosition());

            telemetry.addData("Ticks per second", ticksPerSec);

            telemetry.addLine("Speed Lock: " + (fullSpeedLock ? "On" : "Off"));
            telemetry.addLine("Speed Multiplier: " + ((speedMultiplier==1.0d) ? "Toggle On" : "Toggle Off"));
            telemetry.addLine("Second Outtake Motor Power: " + outtakeTwoPower);
            telemetry.addLine("Intake Power: " + intakePower);
            telemetry.addLine("Intake Direction: " + ((intakeDirection==1)? "In" : "Out"));
            telemetry.addLine("Servo Position: "+servoPosition);

            telemetry.addData("TPS at a certain moment", outtakeMotorLine);

            /*
            telemetry.addLine("Front Left Current: " + leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("Front Right Current: " + rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("Back Left Current: " + leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("Back Right Current: " + rightRear.getCurrent(CurrentUnit.AMPS));
             */

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