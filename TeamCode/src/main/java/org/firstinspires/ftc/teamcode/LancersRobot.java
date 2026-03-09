package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public final Limelight3A limelight;
    public final LimelightWrapper limelightWrapper;
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
    private boolean redMode;
    private boolean autonMode;

    public boolean turretMode = true;

    // ---- LIMELIGHT DEBUG ----
    private double llTy = 0;
    private double llBaseAngle = 0;
    private double llCorrectedAngle = 0;
    private boolean llActive = false;

    public final static double CAMERA_TO_PIVOT_IN = 5.75;
    public final static double PIVOT_TO_ROBOT_CENTER_IN = 1.59;

    // ---- STATE ----
    private double servoPosition = 0;
    private double outtakeVelocity = 0.0;
    private double outtakeTwoPower = -1.0;
    private double intakePower = 0.0;
    private double ticksPerSec;

    private boolean adjustedVelocity;

    private HardwareMap hardwareMap;

    private InterpLUT servoLUT;

    // PIDF For Outtake
    public PIDFCoefficients closeShootingPIDF = new PIDFCoefficients(30, 0, 0, 10);
    public PIDFCoefficients farShootingPIDF = new PIDFCoefficients(60.000, 0, 0, 19.000);
    public PIDFCoefficients underShootingPIDF = new PIDFCoefficients(22, 0, 0, 7);

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
        outtakeRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, LancersBotConfig.PINPOINT);
        odo.setOffsets(5.5, -5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        if (outtakeVelIsOn) {
            outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, closeShootingPIDF);
        } else {
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
        outtakeRotationMotor.setPower(power);
    }

    public DcMotorEx getOuttakeMotorTwo() {
        return this.outtakeMotorTwo;
    }

    public double getXToRed() {
        return 137 - odo.getPosY(DistanceUnit.INCH);
    }

    public double getXToBlue() {
        return odo.getPosY(DistanceUnit.INCH) - 7;
    }

    public double getY() {
        return 133 - odo.getPosX(DistanceUnit.INCH);
    }

    private double wrapRelativeTo(double angle, double center) {
        double diff = angle - center;
        while (diff >= -45) diff -= 360.0;
        while (diff < -45) diff += 360.0;
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

    public double getIntegratedAngle(boolean isRed) {
        double baseAngle = isRed ? getAngleToRed() : getAngleToBlue();

        double ty = limelightWrapper.getTy();
        llTy = ty;
        llBaseAngle = baseAngle;
        llActive = Math.abs(ty) < 10 && Math.abs(ty) > 1;

        llCorrectedAngle = baseAngle;
        return wrapRelativeTo(baseAngle, 0);
    }

    public void aimTurretToAngle(double desiredTurretDeg, double maxPower, double deadbandDeg) {
        targetTicks = turretDegToTicks(desiredTurretDeg);
        currentTicks = -outtakeRotationMotor.getCurrentPosition();
        errorTicks = targetTicks - currentTicks;
        errorDeg = errorTicks / -turretTicksPerDegree;

        if (Math.abs(errorDeg) < deadbandDeg) {
            outtakeRotationMotor.setPower(0);
            return;
        }

        double[] step = {1, 0.8, 0.4, 0.2, 0.10, 0};
        double power = 0;

        if (Math.abs(errorTicks) >= 500) power = step[0];
        if (Math.abs(errorTicks) < 500 && Math.abs(errorTicks) > 200) power = step[1];
        if (Math.abs(errorTicks) < 200 && Math.abs(errorTicks) > 100) power = step[2];
        if (Math.abs(errorTicks) < 100 && Math.abs(errorTicks) > 70) power = step[3];
        if (Math.abs(errorTicks) < 70 && Math.abs(errorTicks) > 30) power = step[4];
        if (Math.abs(errorTicks) < 30) power = step[5];

        if (errorTicks < 0) outtakeRotationMotor.setPower(power);
        if (errorTicks > 0) outtakeRotationMotor.setPower(-power);

        telemetry.addData("Rotation Power:", power);
    }

    public void holdTurretAngle(double relativeDeg, double maxPower) {
        targetTicks = turretDegToTicks(relativeDeg);
        outtakeRotationMotor.setTargetPosition((int) (-targetTicks));
        outtakeRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeRotationMotor.setPower(maxPower);
    }

    public void resetOuttakeRotationMotorPosition() {
        outtakeRotationMotorPosition = outtakeRotationMotor.getCurrentPosition();
    }

    public double getCurrentTurretDeg() {
        double currentTicks = -outtakeRotationMotor.getCurrentPosition();
        return (currentTicks - turretTicksIntercept) / turretTicksPerDegree;
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

    public void setAdjustedOuttakeVelocity() {
        adjustedVelocity = !adjustedVelocity;
    }

    public void autoAdjustOuttakeVelocity() {
        if (adjustedVelocity) {
            if (odo.getPosY(DistanceUnit.INCH) < 48) {
                outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, farShootingPIDF);
                outtakeMotor.setVelocity(1240);
                setServoPosition(0.8);
            } else if (odo.getPosY(DistanceUnit.INCH) >= 48) {
                outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, closeShootingPIDF);
                outtakeMotor.setVelocity(1260);
                setServoPosition(1);
            }
        } else {
            outtakeMotor.setVelocity(0);
        }
    }

    public void setOuttakePIDF(PIDFCoefficients coefficients) {
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
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
            double multiplier = limelightWrapper.getDistanceToTag(redMode);
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

    public void setTurretMode() {
        turretMode = !turretMode;
        if (turretMode) outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, underShootingPIDF);
        else outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, farShootingPIDF);
    }

    private boolean setOriginalTime = false;
    private double originalTime;

    public void setOriginalTime() {
        this.setOriginalTime = true;
        this.originalTime = System.currentTimeMillis();
    }

    public void relocalizeRobot() {
        if (redMode) {
            odo.setPosX(144 - 7.75, DistanceUnit.INCH);
        } else {
            odo.setPosX(7.75, DistanceUnit.INCH);
        }
        odo.setPosY(7.86025, DistanceUnit.INCH);
        odo.setHeading(90, AngleUnit.DEGREES);
    }

    public void aimReset() {
        if (setOriginalTime) {
            if (result != null && result.isValid()) {
                setOriginalTime = false;
            }
            double timeDiff = System.currentTimeMillis() - originalTime;
            if (timeDiff >= 2000) {
                outtakeRotationMotor.setPower(0);
                setOriginalTime = false;
            } else if (timeDiff >= 1000) {
                outtakeRotationMotor.setPower(-0.8);
            } else if (timeDiff >= 0) {
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
    }

    public void sendTelemetry(boolean showVision) {
        telemetry.addLine("----------------------------");
        telemetry.addData("Ticks per second", ticksPerSec);
        telemetry.addData("Target velocity", outtakeVelocity);
        telemetry.addData("Actual velocity", outtakeMotor.getVelocity());
        telemetry.addData("Outtake 2 power", outtakeTwoPower);
        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Outtake rotation motor ticks", -outtakeRotationMotor.getCurrentPosition() - outtakeRotationMotorPosition);
        telemetry.addData("odo X", odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("odo Y", odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("odo heading (raw)", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("----------------------------");
        telemetry.addData("target ticks", targetTicks);
        telemetry.addData("current ticks", currentTicks);
        telemetry.addData("error ticks", errorTicks);
        telemetry.addData("target angle (field)", targetAngle);
        telemetry.addData("robot angle", robotAngle);
        telemetry.addData("desired turret deg", desiredTurretDeg);
        telemetry.addData("error deg", errorDeg);
        telemetry.addLine("rotation direction: " + ((errorTicks < 0) ? "clockwise" : "counter-clockwise"));
        telemetry.addLine("----------------------------");
        telemetry.addData("Distance to goal", "%.1f in", distanceToGoal());
        // NO telemetry.update() here — opmode calls it
    }

    // ---- UTILS ----
    public static double respectDeadZones(double input) {
        return (Math.abs(input) < DEAD_ZONE_LIMIT) ? 0.0 : input;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
}