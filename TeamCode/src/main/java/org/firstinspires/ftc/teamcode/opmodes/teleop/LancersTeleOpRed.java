package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LancersRobot;
import org.firstinspires.ftc.teamcode.LancersTeleOpController;

@TeleOp(name = "LancersTeleOp | Red")
public class LancersTeleOpRed extends LinearOpMode {

    private boolean started = false;
    private boolean calibrated = false;

    private double turretOffset = 0;
    private boolean outtakeTwoReversed = true;
    private double outtakeTwoPower = 0;
    private boolean farShot = false;

    // TODO: tune these two velocities
    private static final double CLOSE_SHOT_VELOCITY = 960;
    private static final double FAR_SHOT_VELOCITY   = 1240;

    private ElapsedTime limelightTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        LancersRobot robot = new LancersRobot(hardwareMap, telemetry, true, true, false);
        LancersTeleOpController controller = new LancersTeleOpController(hardwareMap, robot);

        waitForStart();
        if (isStopRequested()) return;

        limelightTimer.reset();

        // -------------------------
        // STARTUP 360° SWEEP TO CALIBRATE TURRET TO TAG 24
        // -------------------------
        {
            double sweepStart = robot.getIntegratedAngle(true);
            double sweepSpeed = 0.3;
            double degreesPerLoop = 2.0;
            double totalSwept = 0;

            while (opModeIsActive() && totalSwept < 360 && !calibrated) {

                robot.aimTurretToAngle(sweepStart + totalSwept, sweepSpeed, 5);
                totalSwept += degreesPerLoop;

                LLResult result = robot.limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    boolean tag24Visible = result.getFiducialResults().stream()
                            .anyMatch(f -> f.getFiducialId() == 24);

                    if (tag24Visible) {
                        calibrated = true;
                        result.getFiducialResults().stream()
                                .filter(f -> f.getFiducialId() == 24)
                                .findFirst()
                                .ifPresent(tag -> {
                                    double tx = tag.getTargetXDegrees();
                                    turretOffset = tx;
                                    telemetry.addData("Startup Calibration", "Tag 24 found! tx: %.2f° | offset: %.2f°", tx, turretOffset);
                                });
                    }
                }

                if (!calibrated) {
                    telemetry.addData("Startup Calibration", "Scanning... %.1f° / 360°", totalSwept);
                }

                telemetry.update();
            }

            if (!calibrated) {
                telemetry.addData("Startup Calibration", "Tag 24 not found — continuing on odometry");
                telemetry.update();
            }
        }

        // -------------------------
        // AUTO-START MOTORS
        // -------------------------
        robot.setOuttakePIDF(robot.closeShootingPIDF);
        robot.setOuttakeVelocity(CLOSE_SHOT_VELOCITY);
        robot.setIntake(0.85);
        outtakeTwoPower = 0.9;
        robot.setOuttakeTwoPower(outtakeTwoReversed ? -outtakeTwoPower : outtakeTwoPower);

        while (opModeIsActive()) {

            if (!started) {
                started = true;
                robot.odo.setPosX(120, DistanceUnit.INCH);
                robot.odo.setPosY(24, DistanceUnit.INCH);
                robot.odo.setHeading(90, AngleUnit.DEGREES);
            }

            // -------------------------
            // GAMEPAD 1 B — TOGGLE FAR/CLOSE SHOT VELOCITY
            // -------------------------
            if (gamepad1.b) {
                farShot = !farShot;
                if (farShot) {
                    robot.setOuttakePIDF(robot.farShootingPIDF);
                    robot.setOuttakeVelocity(FAR_SHOT_VELOCITY); // TODO: tune far shot ticks/sec
                } else {
                    robot.setOuttakePIDF(robot.closeShootingPIDF);
                    robot.setOuttakeVelocity(CLOSE_SHOT_VELOCITY); // TODO: tune close shot ticks/sec
                }
            }

            // -------------------------
            // GAMEPAD 2 DPAD — MANUAL TURRET TRIM
            // -------------------------
            if (gamepad2.dpad_left) {
                turretOffset -= 1.0;
            } else if (gamepad2.dpad_right) {
                turretOffset += 1.0;
            }

            // -------------------------
            // GAMEPAD 2 A — RELOCK ONTO TAG 24
            // -------------------------
            if (gamepad2.a) {

                double sweepStart = robot.getIntegratedAngle(true);
                double sweepSpeed = 0.3;
                double degreesPerLoop = 2.0;
                double totalSwept = 0;
                boolean tagFound = false;

                while (opModeIsActive() && totalSwept < 360 && !tagFound) {

                    robot.aimTurretToAngle(sweepStart + totalSwept, sweepSpeed, 5);
                    totalSwept += degreesPerLoop;

                    LLResult result = robot.limelight.getLatestResult();

                    if (result != null && result.isValid()) {
                        boolean tag24Visible = result.getFiducialResults().stream()
                                .anyMatch(f -> f.getFiducialId() == 24);

                        if (tag24Visible) {
                            tagFound = true;
                            result.getFiducialResults().stream()
                                    .filter(f -> f.getFiducialId() == 24)
                                    .findFirst()
                                    .ifPresent(tag -> {
                                        double tx = tag.getTargetXDegrees();
                                        turretOffset = tx;
                                        telemetry.addData("LL Tag 24 Relock", "tx: %.2f° | offset: %.2f°", tx, turretOffset);
                                    });
                            telemetry.addData("LL Sweep", "Tag 24 found at %.1f° swept", totalSwept);
                        }
                    }

                    if (!tagFound) {
                        telemetry.addData("LL Sweep", "Scanning... %.1f° / 360°", totalSwept);
                    }

                    telemetry.update();
                    controller.loop(gamepad1, gamepad2);
                }
            }

            // -------------------------
            // LIMELIGHT APRILTAG CORRECTION (every 5 seconds)
            // -------------------------
            if (limelightTimer.seconds() > 5) {

                LLResult result = robot.limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    boolean tag24Visible = result.getFiducialResults().stream()
                            .anyMatch(f -> f.getFiducialId() == 24);

                    if (tag24Visible) {
                        result.getFiducialResults().stream()
                                .filter(f -> f.getFiducialId() == 24)
                                .findFirst()
                                .ifPresent(tag -> {
                                    double tx = tag.getTargetXDegrees();
                                    turretOffset = tx;
                                    telemetry.addData("LL Tag 24 correction", "tx: %.2f° | offset: %.2f°", tx, turretOffset);
                                });
                    } else {
                        turretOffset = 0;
                        telemetry.addData("LL Tag 24", "Not visible — clearing offset");
                    }
                }

                limelightTimer.reset();
            }

            // -------------------------
            // TURRET CONTROL
            // -------------------------
            if (robot.turretMode) {
                double robotHeading = robot.getIntegratedAngle(true);
                double targetTurretAngle = robotHeading + turretOffset;
                robot.aimTurretToAngle(targetTurretAngle, 0.8, 5);
                telemetry.addData("Turret Target", "%.2f°", targetTurretAngle);
            } else {
                robot.holdTurretAngle(turretOffset, 0.6);
            }

            telemetry.addData("Shot Mode", farShot ? "FAR" : "CLOSE");
            telemetry.addData("Turret Offset", "%.2f°", turretOffset);

            // ONE telemetry.update() at the very end
            telemetry.update();
            controller.loop(gamepad1, gamepad2);
        }
    }
}