package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "AprilTagScannerForAuton", group = "Vision")
public class AprilTagScannerForAuton extends OpMode {

    private static final int TARGET_TAG_ID = 20;

    // Hardware
    private Limelight3A limelight;
    private DcMotorEx turret;

    // Tuning
    private double kP = 0.020;      // power per degree of tx
    private double kD = 0.0015;     // damping
    private double kS = 0.05;       // friction assist
    private double maxPower = 0.45;

    private double deadbandDeg = 1.0;   // if |tx| < this, stop (prevents jitter)
    private long maxStaleMs = 120;      // ignore old Limelight frames

    // State for derivative
    private double lastTx = 0.0;
    private long lastTimeNs = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(DcMotorEx.class, "outtakeRotationMotor");

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.setPollRateHz(100);  // request data up to 100Hz :contentReference[oaicite:1]{index=1}
        limelight.start();             // start polling :contentReference[oaicite:2]{index=2}
        limelight.pipelineSwitch(0);   // make sure pipeline 0 is your AprilTag pipeline :contentReference[oaicite:3]{index=3}

        lastTimeNs = System.nanoTime();

        telemetry.addLine("Aiming ONLY when AprilTag ID 20 is visible.");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult(); // :contentReference[oaicite:4]{index=4}

        long nowNs = System.nanoTime();
        double dt = (nowNs - lastTimeNs) / 1e9;
        if (dt <= 0) dt = 1e-3;
        lastTimeNs = nowNs;

        double turretPower = 0.0;
        boolean foundTag20 = false;
        double tx = 0.0;

        if (result != null && result.isValid() && result.getStaleness() <= maxStaleMs) { // :contentReference[oaicite:5]{index=5}
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults(); // :contentReference[oaicite:6]{index=6}

            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() == TARGET_TAG_ID) { // :contentReference[oaicite:7]{index=7}
                    foundTag20 = true;
                    tx = f.getTargetXDegrees();            // tag-specific tx (degrees) :contentReference[oaicite:8]{index=8}
                    break;
                }
            }

            if (foundTag20) {
                // Deadband: stop when basically centered
                if (Math.abs(tx) > deadbandDeg) {
                    double dTx = (tx - lastTx) / dt;

                    double cmd = (kP * tx) + (kD * dTx);
                    cmd += Math.signum(cmd) * kS;

                    turretPower = Range.clip(cmd, -maxPower, maxPower);
                } else {
                    turretPower = 0.0;
                }

                lastTx = tx;

                telemetry.addData("Tag", "FOUND ID 20");
                telemetry.addData("tx (deg)", tx);
            } else {
                turretPower = 0.0;
                telemetry.addData("Tag", "ID 20 NOT FOUND");
            }

            telemetry.addData("stale (ms)", result.getStaleness());
        } else {
            turretPower = 0.0;
            telemetry.addData("Limelight", "No valid data / stale");
        }

        turret.setPower(turretPower);

        telemetry.addData("turret power", turretPower);
        telemetry.addData("turret enc", turret.getCurrentPosition());
        telemetry.update();
    }
}
