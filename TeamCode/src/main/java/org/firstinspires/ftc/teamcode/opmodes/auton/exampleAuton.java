package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "exampleAuton")
public class exampleAuton extends OpMode {
    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION -> END POSITION
        // DRIVE -> MOVEMENT STATE
        // SHOOT -> ATTEMPT TO SCORE THE ARTIFACT

        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_TIME,
        DRIVE_SHOOTPOS_LEAVEPOS
    }

    PathState pathState;

    // The poses are estimates
    private final Pose startPose = new Pose(20, 122, Math.toRadians(138));
    private final Pose shootPose = new Pose(48, 96, Math.toRadians(138));
    private final Pose leavePose = new Pose(32, 80, Math.toRadians(138));

    private PathChain driveStartPosShootPos, driveShootPosLeavePose;

    public void buildPaths() {
        startPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootPosToLeavePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(startPosToShootPos, true);
                pathState = PathState.SHOOT_TIME;

                break;
            case SHOOT_TIME:
                // add flywheel logic here

                if (!follower.isBusy()) {
                    telemetry.addLine("Path 1 Completed");
                }
                break;
            case DRIVE_SHOOTPOS_LEAVEPOS:
                follower.followPath(shootPosToLeavePos, true);
                if (!follower.isBusy()) {
                    telemetry.addLine("Auton Complete");
                }
            default:
                telemetry.addLine("No State Commanded");
                break;


        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        // add in other mechanisms that need to be initialized (limelight for example)

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());

    }

}
