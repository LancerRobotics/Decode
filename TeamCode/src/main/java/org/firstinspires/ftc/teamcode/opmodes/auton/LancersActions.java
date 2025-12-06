package org.firstinspires.ftc.teamcode.opmodes.auton;

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

    public Intake(boolean running, HardwareMap hardwareMap) {
        this.running = running;
        this.hardwareMap = hardwareMap;

        this.intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.INTAKE_MOTOR);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (running) {
            intakeMotor.setPower(0.75);
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
    private DcMotorEx outtakeMotorTwo;

    public OuttakeMotor(boolean running, HardwareMap hardwareMap) {
        this.running = running;
        this.hardwareMap = hardwareMap;

        this.outtakeMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR);
        //this.outtakeMotorTwo = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR_TWO);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (running) {
            outtakeMotor.setPower(1);
            //outtakeMotorTwo.setPower(-1);
            //outtakeMotorTwo.setPower(-0.5);
        } else {
            outtakeMotor.setPower(0);
            //outtakeMotorTwo.setPower(0);
        }
        return false;
    }
}

class OuttakeMotorTwo implements Action {
    private HardwareMap hardwareMap;
    private boolean running = false;

    private DcMotorEx outtakeMotorTwo;

    public OuttakeMotorTwo(boolean running, HardwareMap hardwareMap) {
        this.running = running;
        this.hardwareMap = hardwareMap;

        this.outtakeMotorTwo = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.OUTTAKE_MOTOR_TWO);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (running) {
            //outtakeMotorTwo.setPower(-1);
            outtakeMotorTwo.setPower(-0.5);
        } else {
            outtakeMotorTwo.setPower(0);
        }
        return false;
    }
}

class OuttakeServo implements Action {
    private HardwareMap hardwareMap;
    private Servo outtakeServo;
    private boolean closeGate;

    public OuttakeServo(boolean closeGate, HardwareMap hardwareMap) {
        // false for start position
        //true for end position
        this.closeGate = closeGate;

        this.hardwareMap = hardwareMap;

        this.outtakeServo = hardwareMap.servo.get(LancersBotConfig.OUTTAKE_SERVO);
        outtakeServo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (this.closeGate) {
            this.outtakeServo.setPosition(0.55);
        }
        else {
            this.outtakeServo.setPosition(0.35);
        }
        return false;
    }
}