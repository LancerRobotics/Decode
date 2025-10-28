package org.firstinspires.ftc.teamcode.RRAutons;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.LancersBotConfig;

class Movement implements Action {
    private HardwareMap hardwareMap;
    private double time;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    public Movement(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.leftFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        this.leftRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        this.rightFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        this.rightRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        leftFront.setPower(0.2);
        leftRear.setPower(0.2);
        rightFront.setPower(-0.2);
        rightRear.setPower(-0.2);

        return false;

    }

    public void stopMotors(){
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}

class Intake implements Action {
    private HardwareMap hardwareMap;
    private boolean running = false;

    private DcMotorEx intakeMotor;

    public Intake(boolean running) {
        this.running = running;
        this.hardwareMap = hardwareMap;

        this.intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.INTAKE_MOTOR);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (running) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
        return false;
    }
}

class OuttakeMotor implements Action {
    private HardwareMap hardwareMap;
    private boolean running = false;

    private DcMotorEx outtakeMotor;
    private Servo outtakeServo;

    public OuttakeMotor(boolean running) {
        this.running = running;
        this.hardwareMap = hardwareMap;

        this.outtakeMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (running) {
            outtakeMotor.setPower(1);
        } else {
            outtakeMotor.setPower(0);
        }
        return false;
    }
}

class OuttakeServo implements Action {
    private HardwareMap hardwareMap;
    private double pos = 0;

    private Servo outtakeServo;

    public OuttakeServo(double pos) {
        // false for start position
        //true for end position

        this.pos = pos;
        this.hardwareMap = hardwareMap;

        this.outtakeServo = hardwareMap.servo.get(LancersBotConfig.OUTTAKE_SERVO);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        outtakeServo.setPosition(pos);
        return false;
    }
}