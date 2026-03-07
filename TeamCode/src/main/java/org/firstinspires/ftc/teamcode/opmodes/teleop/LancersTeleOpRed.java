package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LancersRobot;
import org.firstinspires.ftc.teamcode.LancersTeleOpController;

@TeleOp(name = "LancersTeleOp | Red")
public class LancersTeleOpRed extends LinearOpMode {

    private boolean started = false;

    private double turretOffset = -360;

    private ElapsedTime limelightTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        LancersRobot robot = new LancersRobot(hardwareMap, telemetry, true, true, false);
        LancersTeleOpController controller = new LancersTeleOpController(hardwareMap, robot);

        waitForStart();
        if (isStopRequested()) return;

        limelightTimer.reset();

        while (opModeIsActive()) {

            if (!started) {
                started = true;
                robot.odo.setPosX(120, DistanceUnit.INCH);
                robot.odo.setPosY(24, DistanceUnit.INCH);
                robot.odo.setHeading(90, AngleUnit.DEGREES);
            }

            //auto corrects with limelight every 5 seconds
            if (limelightTimer.seconds() > 5) {

                LLResult result = robot.limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    Pose3D pose = result.getBotpose();

                    if (pose != null) {

                        double tagX = pose.getPosition().x;
                        double tagY = pose.getPosition().y;
                        double tagHeading = 90 + pose.getOrientation().getYaw(AngleUnit.DEGREES);


                        double odoX = robot.odo.getPosX(DistanceUnit.INCH);
                        double odoY = robot.odo.getPosY(DistanceUnit.INCH);
                        double odoHeading = robot.odo.getHeading(AngleUnit.DEGREES);

                        double posError = Math.sqrt(
                                Math.pow(tagX - odoX, 2) +
                                        Math.pow(tagY - odoY, 2)
                        );
                        double headingError = Math.abs(tagHeading - odoHeading);


                        if (posError < 24 && headingError < 30) {

                            robot.odo.setPosX(tagX, DistanceUnit.INCH);
                            robot.odo.setPosY(tagY, DistanceUnit.INCH);
                            robot.odo.setHeading(tagHeading, AngleUnit.DEGREES);

                            telemetry.addData("LL Correction", "Applied | pos err: %.1f\" hdg err: %.1f°", posError, headingError);
                        } else {
                            telemetry.addData("LL Correction", "Rejected | pos err: %.1f\" hdg err: %.1f°", posError, headingError);
                        }

                        //locks turret onto apriltag regardless of what the odometry wants
                        double tx = result.getTx();
                        double robotHeading = robot.getIntegratedAngle(true);

                        turretOffset = robotHeading + tx;

                        telemetry.addData("LL Tag Lock", "tx: %.2f° | new offset: %.2f°", tx, turretOffset);
                    }
                }

                limelightTimer.reset();
            }


            if (robot.turretMode) {
                double robotHeading = robot.getIntegratedAngle(true);
                double targetTurretAngle = robotHeading + turretOffset;

                robot.aimTurretToAngle(targetTurretAngle, 0.8, 5);
                telemetry.addData("Turret Target", "%.2f°", targetTurretAngle);
            } else {
                robot.holdTurretAngle(turretOffset, 0.6);
            }

            telemetry.update();
            controller.loop(gamepad1, gamepad2);
        }
    }
}

//the turret solely works off goon power so if it stops working, you know what to do...