package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.LimelightWrapper;


public class LancersRobot {

    // ---- CONSTANTS ----
    private static final double TICKS_PER_REV = 1534.4;
    public static final double DEAD_ZONE_LIMIT = 0.15;

    private double turretTicksPerDegree = -4.6733;
    private double turretTicksIntercept = -3.6;
    private double turretZeroOffset = 0;

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
    public double outtakeRotationMotorPosition;
    private final Servo outtakeServo;

    public GoBildaPinpointDriver odo;

    // ---- VISION ----
    private final Limelight3A limelight;
    private final LimelightWrapper limelightWrapper;
    private LLResult result;

    // ---- TELEMETRY ----
    private final Telemetry telemetry;

    // ---- TURRET TRACKING ----
    private double targetTicks;
    private double currentTicks;
    private double errorTicks;
    private double errorDeg;
    private double desiredTurretDeg;
    private double targetAngle;
    private double robotAngle;
    private boolean redMode; // true for red, false for blue
    private boolean autonMode;

    // ---- LIMELIGHT DEBUG ----
    private double llTy = 0;
    private double llBaseAngle = 0;
    private double llCorrectedAngle = 0;
    private boolean llActive = false;

    // ---- STATE ----
    private double servoPosition = 0;
    private double outtakeVelocity = 0.0;
    private double outtakeTwoPower = -1.0;
    private double intakePower = 0.0;
    private double ticksPerSec;

    private HardwareMap hardwareMap;

    private InterpLUT servoLUT;

    public LancersRobot(HardwareMap hardwareMap, Telemetry telem, boolean outtakeVelIsOn, boolean redMode, boolean autonMode) {

        servoLUT = new InterpLUT();
        //servoLUT.add(distance (inches), servo position);
        //servoLUT.createLUT();

        this.hardwareMap = hardwareMap;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telem, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        limelight = hardwareMap.get(Limelight3A.class, LancersBotConfig.LIMELIGHT);
        limelight.pipelineSwitch(9);
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
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR);
        outtakeMotorTwo = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR_TWO);
        outtakeRotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_ROTATION_MOTOR);

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeRotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // resets stored encoder values and stops motor
        outtakeRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         odo = hardwareMap.get(GoBildaPinpointDriver.class, LancersBotConfig.PINPOINT);

         odo.setOffsets(5.5, -5, DistanceUnit.INCH); // these values change depending on the center of mass
         odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
         odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

         odo.resetPosAndIMU();

        if (outtakeVelIsOn) {
            outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients farShootingPIDF = new PIDFCoefficients(140.00, 0, 0, 28.00); // far shooting

            PIDFCoefficients otherPIDF = new PIDFCoefficients(120, 1, 10, 20.5);

            PIDFCoefficients tempPIDF = new PIDFCoefficients(120, 0, 0, 3); // around 1080?
            //PIDFCoefficients pidf = new PIDFCoefficients(70, 0, 0, 30);

            // 2/21/26
            PIDFCoefficients tempPIDF2 = new PIDFCoefficients(68, 0, 0, 24.4);
            outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, tempPIDF);
        }
        else {
            outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        outtakeServo = hardwareMap.servo.get(LancersBotConfig.OUTTAKE_SERVO);
        outtakeServo.resetDeviceConfigurationForOpMode();
        outtakeServo.setPosition(servoPosition);

        outtakeRotationMotorPosition = 0;

        this.redMode = redMode;
        this.autonMode = autonMode;
    }

    public void setOuttakeRotationMotor(double power) {
        outtakeRotationMotor.setPower(power); //temp
    }
    public DcMotorEx getOuttakeMotorTwo() {
        return this.outtakeMotorTwo;
    }

    /***
     * In ftc, x and y are switched, so the odo.getPosY()
     * and odo.getPosX() functions are also switched.
     *
     * nevermind ignore this
     ***/

    public double getXToRed() {
        return 131 - odo.getPosY(DistanceUnit.INCH);
    }
    public double getXToBlue() {
        return odo.getPosY(DistanceUnit.INCH) - 13;
    }
    public double getY() {
        return 133 - odo.getPosX(DistanceUnit.INCH);
    }

    private double wrapRelativeTo(double angle, double center) {
        double diff = angle - center;
        while (diff >= -50) diff -= 360.0;
        while (diff < -50) diff += 360.0;
        //diff = Math.max(-45.0, Math.min(45.0, diff));
        return center + diff;
    }


    public void changeTurretOffset(double amt) {
        turretZeroOffset += amt;
    }

    public double getAngleToRed() {

        double currentX = odo.getPosX(DistanceUnit.INCH);
        double currentY = odo.getPosY(DistanceUnit.INCH);

        double targetX = 131;
        double targetY = 133;

        double dx = targetX - currentX;
        double dy = targetY - currentY;

        targetAngle = Math.toDegrees(Math.atan2(dy, dx));
        robotAngle = odo.getHeading(AngleUnit.DEGREES);

        double desired = targetAngle - robotAngle - turretZeroOffset;
        desired = wrapRelativeTo(desired, 0);

        desiredTurretDeg = desired;
        return desiredTurretDeg;
    }
    public double getAngleToBlue() {

        double currentX = odo.getPosX(DistanceUnit.INCH);
        double currentY = odo.getPosY(DistanceUnit.INCH);

        double targetX = 13;
        double targetY = 133;

        double dx = targetX - currentX;
        double dy = targetY - currentY;

        targetAngle = Math.toDegrees(Math.atan2(dy, dx));
        robotAngle = odo.getHeading(AngleUnit.DEGREES);

        double desired = targetAngle - robotAngle - turretZeroOffset;
        desired = wrapRelativeTo(desired, 0);

        desiredTurretDeg = desired;
        return desiredTurretDeg;
    }

    private double turretDegToTicks(double deg) {
        return (turretTicksPerDegree * deg) + turretTicksIntercept;
    }

    //Adds Limelight tracking to the existing odo logic for turret targeting
    public double getIntegratedAngle(boolean isRed) {
        double baseAngle = isRed ? getAngleToRed() : getAngleToBlue();

        double ty = limelightWrapper.getTy();
        // ty represents the horizontal angular error (limelight is sideways, so ty = horizontal)

        llTy = ty;
        llBaseAngle = baseAngle;
        llActive = Math.abs(ty) < 10 && Math.abs(ty) > 1;

        if (llActive) {
            baseAngle -= ty * 0.6;
        }

        llCorrectedAngle = baseAngle;
        return wrapRelativeTo(baseAngle, 0);


        /*
        double finalTargetAngle = isRed ? getAngleToRed() : getAngleToBlue();

        double currentTicks = -outtakeRotationMotor.getCurrentPosition();
        double currentAngle = (currentTicks - turretTicksIntercept) / turretTicksPerDegree;
        robotAngle = odo.getHeading(AngleUnit.DEGREES);

        double poseOffset = 0;

        if (limelightWrapper.getBotPose() != null) {
            poseOffset = limelightWrapper.getTy();
            double fieldAngle = robotAngle + currentAngle + poseOffset;
            finalTargetAngle = wrapDegrees(fieldAngle - turretZeroOffset);
        }

        //uncomment this if we get 360 degree movement
        //if (finalTargetAngle > 175) finalTargetAngle = 175;
        //if (finalTargetAngle < -175) finalTargetAngle = -175;

        return finalTargetAngle;

         */
    }

    public void aimTurretToAngle(double desiredTurretDeg, double kP, double maxPower, double deadbandDeg) {

        targetTicks = turretDegToTicks(desiredTurretDeg);

        currentTicks = -outtakeRotationMotor.getCurrentPosition();

        errorTicks = targetTicks - currentTicks; // amount of ticks off
        errorDeg = errorTicks / -turretTicksPerDegree; // amount of degrees off



        if (Math.abs(errorDeg) < deadbandDeg) {
            outtakeRotationMotor.setPower(0);
            return;
        }

        // changes power based on delta value of ticks
        double power = -kP * errorTicks;

        if (power > maxPower) power = maxPower;
        if (power < -maxPower) power = -maxPower;

        outtakeRotationMotor.setPower(power);

        telemetry.addData("Rotation Power:", power);
    }

    public void holdTurretAngle(double relativeDeg, double maxPower) { // USED FOR AUTON

        targetTicks = turretDegToTicks(relativeDeg);

        outtakeRotationMotor.setTargetPosition((int)(-targetTicks));
        outtakeRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeRotationMotor.setPower(maxPower);
    }



    public void resetOuttakeRotationMotorPosition () {
        outtakeRotationMotorPosition = outtakeRotationMotor.getCurrentPosition();
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

    public void setOuttakePower(double power) {
        outtakeMotor.setPower(power);
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
            multiplier = limelightWrapper.getDistanceToTag(redMode);
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

    public double distanceToGoal() {
        double targetY = 133;
        double targetX = (redMode) ? 133 : 13;
        double dx = targetX - odo.getPosX(DistanceUnit.INCH);
        double dy = targetY - odo.getPosY(DistanceUnit.INCH);
        return Math.hypot(dx, dy);
    }

    public void autoAdjustServo() {
        double distance = distanceToGoal();

        double servoPos = servoLUT.get(distance);

        servoPos = Math.max(0.0, Math.min(1.0, servoPos));

        setServoPosition(servoPos);

        telemetry.addData("Distance to goal: ", distance);
        telemetry.addData("Servo Pos:", servoPos);
    }

    // ---- UPDATE / TELEMETRY ----
    public void update() {
        ticksPerSec = outtakeMotor.getVelocity();
        result = limelight.getLatestResult();

        odo.update();

        //autoAdjustServo();


        // For Teleop Only
        if (!autonMode) {
            if (redMode) {
                //aimTurretToAngle(getAngleToRed(), 0.009, 1, 0.5);
                aimTurretToAngle(getIntegratedAngle(true), 0.015, 0.9, 0.25);
            } else {
                //aimTurretToAngle(getAngleToBlue(), 0.009, 1, 0.5);
                aimTurretToAngle(getIntegratedAngle(false), 0.015, 0.9, 0.25);
            }
        }

        // For auton, run holdTurretAngle(degree, max_power) to keep the turret pointed at a certain angle relative to the robot




    }

    public void sendTelemetry(boolean showVision) {


        telemetry.addLine("----------------------------");

        // LIMELIGHT TESTING
        /*
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
         */

        /*
        // INTAKE/OUTTAKE POWER/VEL TESTING
        telemetry.addData("Ticks per second", ticksPerSec);
        telemetry.addData("Target velocity", outtakeVelocity);
        telemetry.addData("Actual velocity", outtakeMotor.getVelocity());
        telemetry.addData("Outtake 2 power", outtakeTwoPower);
        telemetry.addData("Intake power", intakePower);

        //telemetry.addData("Servo position", servoPosition);


         */
        telemetry.addData("Outtake rotation motor ticks", -outtakeRotationMotor.getCurrentPosition()-outtakeRotationMotorPosition);

        // position/pinpoint testing
        telemetry.addData("odo X", odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("odo Y", odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("odo heading (raw)", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("----------------------------");

        // turret testing
        telemetry.addData("target ticks", targetTicks);
        telemetry.addData("current ticks", currentTicks);
        telemetry.addData("error ticks", errorTicks);
        telemetry.addData("target angle (field)", targetAngle);
        telemetry.addData("robot angle", robotAngle);
        telemetry.addData("desired turret deg", desiredTurretDeg);
        telemetry.addData("error deg", errorDeg);
        telemetry.addLine("rotation direction: " + ((errorTicks < 0) ? "clockwise" : "counter-clockwise"));


        telemetry.addLine("----------------------------");

        // limelight correction debug
        telemetry.addData("ll ty (raw)", llTy);
        telemetry.addData("ll base angle (odo only)", llBaseAngle);
        telemetry.addData("ll correction active", llActive);
        telemetry.addData("ll correction applied", llActive ? -(llTy * 0.6) : 0.0);
        telemetry.addData("ll corrected angle", llCorrectedAngle);

        telemetry.addLine("----------------------------");

        telemetry.update();
    }

    // ---- UTILS ----
    public static double respectDeadZones(double input) {
        return (Math.abs(input) < DEAD_ZONE_LIMIT) ? 0.0 : input;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
}

