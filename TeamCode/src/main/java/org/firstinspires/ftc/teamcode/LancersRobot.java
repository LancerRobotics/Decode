package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.LimelightWrapper;


public class LancersRobot {

    // ---- CONSTANTS ----
    final double TICKS_PER_REV = 1534.4; // This number can be found on Gobuilda
    public static final double DEAD_ZONE_LIMIT = 0.15d;

    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;

    private final DcMotor intakeMotor;
    private final DcMotorEx outtakeMotor;
    private final DcMotorEx outtakeMotorTwo;
    private final DcMotorEx outtakeRotationMotor;

    private final Servo outtakeServo;

    private Limelight3A limelight;
    private LimelightWrapper myLimelight;

    private LLResult result;

    public static final String TAG = "LancerTeleOp";
    private long currentRunTimeStamp = -1;
    private double servoPosition = 0.0;

    // INTAKE VARIABLES
    private double intakeDirection = 1; // +1 or -1
    private double intakePower = 0; // 0 or 1
    private double outtakeVelocity = 0;
    private double outtakeTwoPower = 0; // 0 or 1

    private HardwareMap hardwareMap;

    private Telemetry telemetry;

    private double outtakeTicksPerSec;
    private double outtakeRPM1;
    private double outtakeTicksPerSec2;
    private double outtakeRPM2;
    private double ticksPerSec;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public LancersRobot(HardwareMap hm, Telemetry telem, Gamepad gp1, Gamepad gp2) {

        this.gamepad1 = gp1;
        this.gamepad2 = gp2;

        this.hardwareMap = hm;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telem, dashboard.getTelemetry());
        this.telemetry.setMsTransmissionInterval(50); // around 20 hZ

        limelight = hardwareMap.get(Limelight3A.class, LancersBotConfig.LIMELIGHT);
        limelight.pipelineSwitch(5); // switch to 8 or 9 later
        myLimelight = new LimelightWrapper(hardwareMap);

        // Get from hardwaremap, initialize variables as DcMotor type
        this.leftFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        //intake
        this.intakeMotor = hardwareMap.dcMotor.get(LancersBotConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //outtake
        this.outtakeMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR);
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.outtakeMotorTwo = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR_TWO);
        outtakeMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        this.outtakeRotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_ROTATION_MOTOR);
        outtakeRotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(100, 0, 0, 19.8);
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        this.outtakeServo = hardwareMap.servo.get(LancersBotConfig.OUTTAKE_SERVO);

        servoPosition = 0;

        outtakeServo.setPosition(0); // servo intital position


        outtakeVelocity = 0;
        outtakeTwoPower = 0;
        intakeDirection = 1;
    }

    public void loop() {
        handleDrivetrain();
        handleIntake();
        handleOuttake();
        handleServo();

        update();
        applyPower();
        sendTelemetry();

    }

    private void handleDrivetrain() {

        double speedMultiplier = gamepad1.a ? 1.0d : 0.8d;

        double ly = -respectDeadZones(gamepad1.left_stick_y) * speedMultiplier;
        double lx =  respectDeadZones(gamepad1.left_stick_x) * speedMultiplier;
        double rx =  respectDeadZones(gamepad1.right_stick_x) * speedMultiplier;

        double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1.0);

        //Calculations, more in depth here:
        //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

        final double frontLeftPower = (ly + lx + rx) / denominator;
        final double backLeftPower = (ly - lx + rx) / denominator;
        final double frontRightPower = (ly - lx - rx) / denominator;
        final double backRightPower = (ly + lx - rx) / denominator;

        // left drive motor powers are reversed
        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

    }

    private void handleIntake() {
        if (gamepad1.leftBumperWasPressed()){
            intakePower = (intakePower == 0) ? 1 : 0;
        }

        if (gamepad1.rightBumperWasPressed()){
            intakeDirection *= -1;
        }
    }

    private void handleOuttake() {
        if (gamepad2.rightBumperWasPressed()){
            outtakeVelocity = (outtakeVelocity == 0) ? 1500 : 0;
        }

        if (gamepad2.leftBumperWasPressed()){
            outtakeTwoPower = (outtakeTwoPower == 0) ? 0.5 : 0;
        }

        if (result != null) {
            outtakeRotationMotor.setPower((Math.abs(result.getTx())/result.getTx())*0.025);
        }

        if (result == null || Math.abs(result.getTx()) <= 0.5) {
            outtakeRotationMotor.setVelocity(0);
        }
    }

    private void handleServo() {
        if (gamepad2.yWasPressed()) {
            servoPosition = (servoPosition == 0) ? 0.5 : 0;
            outtakeServo.setPosition(servoPosition);
        }
    }

    private void applyPower() {
        outtakeMotor.setVelocity(outtakeVelocity);
        outtakeMotorTwo.setPower(outtakeTwoPower);

        intakeMotor.setPower(intakePower * intakeDirection);
    }


    // outtake encoders + limelight data
    public void update() {
        /*
        this.outtakeTicksPerSec = outtakeMotor.getVelocity();
        this.outtakeRPM1 = (outtakeTicksPerSec / TICKS_PER_REV) * 60.0;
        this.outtakeTicksPerSec2 = outtakeMotorTwo.getVelocity();
        this.outtakeRPM2 = (outtakeTicksPerSec2 / TICKS_PER_REV) * 60.0;

         */

        this.ticksPerSec = outtakeMotor.getVelocity(AngleUnit.DEGREES);

        result = limelight.getLatestResult();
    }



    public void sendTelemetry() {
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());

                //double distance = myLimelight.getDistanceToTag();
                //telemetry.addData("Distance (inches)", distance);
            }
            else {
                telemetry.addData("tx", "null");
                telemetry.addData("ty", "null");
                telemetry.addData("Botpose", "null");
                //telemetry.addData("Distance (inches)", null);
            }
        }


        telemetry.addData("Ticks per second", ticksPerSec);

        telemetry.addData("Target velocity", outtakeVelocity);
        telemetry.addData("Actual Velocity", outtakeMotor.getVelocity());

        telemetry.addLine("Speed Multiplier: " + ((this.gamepad1.a) ? "Toggle On" : "Toggle Off"));
        telemetry.addLine("Second Outtake Motor Power: " + outtakeTwoPower);
        telemetry.addLine("Servo Position: "+servoPosition);

        // DEBUGGING TELEMETRY
        /*
        telemetry.addLine("Intake Power: " + intakePower);
        telemetry.addLine("Intake Direction: " + ((intakeDirection==1)? "In" : "Out"));

        telemetry.addData("Outtake 1 RPM: ", outtakeRPM1);
        telemetry.addData("Outtake 2 RPM: ", outtakeRPM2);
         */


        telemetry.update();
    }

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