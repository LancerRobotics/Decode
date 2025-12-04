package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.LimelightWrapper;

@Disabled // TODO: TEMPORARY, REMEMBER TO REMOVE THIS LINE LATER
@Autonomous(name="blueThree_LL")
public class blueThree_LL extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-54, -54, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        LimelightWrapper limelight = new LimelightWrapper(hardwareMap);

        waitForStart();

        // Possible trajectory route
        MecanumDrive.FollowTrajectoryAction trajectory = (MecanumDrive.FollowTrajectoryAction) drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-24, -24))
                .turnTo(Math.toRadians(230))
                .build();

        // required as an parameter for trajectory.run()
        // displays data on dashboard (192.168.43.1:8080/dash)
        TelemetryPacket packet = new TelemetryPacket();

        // runs every tick, checks camera vision to make any pose adjustments
        while (opModeIsActive()) {

            // main source of localization
            drive.updatePoseEstimate();

            // apply vision correction if available
            Pose2d visionPose = limelight.getBotPose();
            if (visionPose != null) {
                drive.localizer.setPose(visionPose);
            }

            boolean stillRunning = trajectory.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet); // sends data to dashboard
            if (!stillRunning) break;

            // displays data on telemetry
            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("X", pose.position.x);
            telemetry.addData("Y", pose.position.y);
            telemetry.addData("Heading", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            packet = new TelemetryPacket(); // reset for next tick
        }

    }
}