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
    private static final double TICKS_PER_REV = 1534.4;
    public static final double DEAD_ZONE_LIMIT = 0.15;

    // ---- DRIVETRAIN ----
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;

    // ---- MECHANISMS ----
    private final DcMotor intakeMotor;
    private final DcMotorEx outtakeMotor;
    private final DcMotorEx outtakeMotorTwo;
    private final DcMotorEx outtakeRotationMotor;
    private final Servo outtakeServo;

    // ---- VISION ----
    private final Limelight3A limelight;
    private final LimelightWrapper limelightWrapper;
    private LLResult result;

    // ---- FTC ----
    private final Telemetry telemetry;

    // ---- STATE ----
    private double servoPosition = 1.0;
    private double outtakeVelocity = 0.0;
    private double outtakeTwoPower =
    private double intakePower = 0. 0.0;0;
    private double ticksPerSec;

    public LancersRobot(HardwareMap hardwareMap, Telemetry telem) {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telem, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        limelight = hardwareMap.get(Limelight3A.class, LancersBotConfig.LIMELIGHT);
        limelight.pipelineSwitch(8);
        /***
         * LIMELIGHT PIPELINES:
         * 0: Purple Balls
         * 1: Green Balls
         * 5: Testing LL tracking at Alan's house
         * 8: LL tracking for blue
         * 9: LL tracking for red
         ***/
        limelight.start();
        limelightWrapper = new LimelightWrapper(hardwareMap);

        leftFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        leftRear  = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        rightRear  = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor = hardwareMap.dcMotor.get(LancersBotConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR);
        outtakeMotorTwo = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR_TWO);
        outtakeRotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_ROTATION_MOTOR);

        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeRotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(100, 0, 0, 19.8);
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        outtakeServo = hardwareMap.servo.get(LancersBotConfig.OUTTAKE_SERVO);
        outtakeServo.setPosition(servoPosition);
    }
    public void setOuttakeRotationMotor(double power) {
        outtakeRotationMotor.setPower(power);
    }
    public DcMotorEx getOuttakeMotorTwo() {
        return this.outtakeMotorTwo;
    }

    // ---- DRIVE ----
    public void driveMecanum(double ly, double lx, double rx, double multiplier) {
        ly *= multiplier;
        lx *= multiplier;
        rx *= multiplier;

        double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1.0);

        double fl = (ly + lx + rx) / denominator;
        double bl = (ly - lx + rx) / denominator;
        double fr = (ly - lx - rx) / denominator;
        double br = (ly + lx - rx) / denominator;

        leftFront.setPower(fl);
        leftRear.setPower(bl);
        rightFront.setPower(fr);
        rightRear.setPower(br);
    }

    // ---- MECHANISMS ----
    public void setIntake(double power) {
        intakePower = power;
        intakeMotor.setPower(power);
    }

    public void setOuttakeVelocity(double velocity) {
        outtakeVelocity = velocity;
        outtakeMotor.setVelocity(velocity);
    }

    public void setOuttakeTwoPower(double power) {
        outtakeTwoPower = power;
        outtakeMotorTwo.setPower(power);
    }

    public void setServoPosition(double position) {
        servoPosition = position;
        outtakeServo.setPosition(position);
    }

    public void aimOuttakeToTx(double deadbandTx) {
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double multiplier;
            multiplier = limelightWrapper.getDistanceToTag();
            telemetry.addData("Distance to Tag:", multiplier);
            if (Math.abs(tx) > deadbandTx) {
                outtakeRotationMotor.setPower(Math.signum(tx) * (multiplier * 0.001));
            } else {
                outtakeRotationMotor.setPower(0);
            }
        } else {
            outtakeRotationMotor.setPower(0);
        }
    }

    private boolean setOriginalTime = false;
    private double originalTime;

    public void setOriginalTime() {
        this.setOriginalTime = true;
        this.originalTime = System.currentTimeMillis();

    }

    public void aimReset() {
        if(setOriginalTime) {

            if (result != null && result.isValid()) {
                setOriginalTime = false;
            }

            double timeDiff = System.currentTimeMillis() - originalTime;
            if(timeDiff >= 2000) {
                outtakeRotationMotor.setPower(0);
                setOriginalTime = false;
            }
            else if(timeDiff >= 1000) {
                outtakeRotationMotor.setPower(-0.8);
            }
            else if(timeDiff >= 0) {
                outtakeRotationMotor.setPower(0.8);
            }
        }
    }

    // ---- UPDATE / TELEMETRY ----
    public void update() {
        ticksPerSec = outtakeMotor.getVelocity(AngleUnit.DEGREES);
        result = limelight.getLatestResult();
    }

    public void sendTelemetry(boolean showVision) {
        if (showVision) {
            if (result != null && result.isValid()) {
                Pose3D pose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", pose.toString());
            } else {
                telemetry.addData("tx", "null");
                telemetry.addData("ty", "null");
                telemetry.addData("Botpose", "null");
            }
        }

        telemetry.addData("Ticks per second", ticksPerSec);
        telemetry.addData("Target velocity", outtakeVelocity);
        telemetry.addData("Actual velocity", outtakeMotor.getVelocity());
        telemetry.addData("Outtake 2 power", outtakeTwoPower);
        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Servo position", servoPosition);

        telemetry.update();
    }

    // ---- UTILS ----
    public static double respectDeadZones(double input) {
        return (Math.abs(input) < DEAD_ZONE_LIMIT) ? 0.0 : input;
    }
}