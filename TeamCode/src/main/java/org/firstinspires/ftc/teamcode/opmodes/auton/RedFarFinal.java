package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.LancersRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedFarFinal", group = "Autonomous")
public class RedFarFinal extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private LancersRobot robot;

    public enum PathState {
        INITIAL_SHOOT,
        COLLECT_1,
        SHOOT_2,
        COLLECT_2,
        WAIT_FIVE_SECONDS,
        SHOOT_3,
        COLLECT_3,
        SHOOT_4,
        LEAVE,
        DONE
    }

    private PathState pathState;
    private boolean justEntered = true;

    // Time Constants
    private static final double SHOOT_SECONDS = 1.0;
    private static final double WAIT_SECONDS = 5.0;

    // Starting Pose
    private final Pose startPose = new Pose(144-66.694, 17.699, Math.toRadians(180-125));

    private PathChain path1, path2, path3, path4, path5, line6, path9;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(144-66.694, 17.699), new Pose(144-53.528, 36.782), new Pose(12.477, 35.751)))
                .setLinearHeadingInterpolation(Math.toRadians(180-125), Math.toRadians(180-180)).build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(144-12.477, 35.751), new Pose(144-37.244, 11.940), new Pose(66.943, 17.943)))
                .setLinearHeadingInterpolation(Math.toRadians(180-180), Math.toRadians(180-125)).build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-66.943, 17.943), new Pose(144-10.601, 12.057)))
                .setLinearHeadingInterpolation(Math.toRadians(180-125), Math.toRadians(180-185)).build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-10.601, 12.057), new Pose(144-66.788, 17.834)))
                .setLinearHeadingInterpolation(Math.toRadians(180-185), Math.toRadians(180-125)).build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(144-66.788, 17.834), new Pose(144-32.544, 11.653), new Pose(10.197, 36.311)))
                .setLinearHeadingInterpolation(Math.toRadians(180-125), Math.toRadians(180-110)).build();

        line6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-10.197, 36.311), new Pose(144-65.409, 19.150)))
                .setLinearHeadingInterpolation(Math.toRadians(180-110), Math.toRadians(180-125)).build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-65.409, 19.150), new Pose(144-49.492, 23.378)))
                .setLinearHeadingInterpolation(Math.toRadians(180-125), Math.toRadians(180-90)).build();
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        justEntered = true;
    }

    private void statePathUpdate() {
        switch (pathState) {
            case INITIAL_SHOOT:
                if (justEntered) {
                    robot.setOuttakeVelocity(1260);
                    robot.setIntake(1); // TURN ON AND LEAVE ON
                    justEntered = false;
                }
                if (pathTimer.getElapsedTimeSeconds() >= 3.0) {
                    robot.setOuttakeTwoPower(1);
                }

                if (pathTimer.getElapsedTimeSeconds() >= 3+SHOOT_SECONDS) {
                    robot.setOuttakeTwoPower(0);
                    setPathState(PathState.COLLECT_1);
                }
                break;

            case COLLECT_1:
                if (justEntered) {
                    follower.followPath(path1, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (justEntered) {
                    follower.followPath(path2, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    robot.setOuttakeTwoPower(1);
                    if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SECONDS) {
                        robot.setOuttakeTwoPower(0);
                        setPathState(PathState.COLLECT_2);
                    }
                }
                break;

            case COLLECT_2:
                if (justEntered) {
                    follower.followPath(path3, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_FIVE_SECONDS);
                }
                break;

            case WAIT_FIVE_SECONDS:
                if (justEntered) {
                    justEntered = false;
                }
                if (pathTimer.getElapsedTimeSeconds() >= WAIT_SECONDS) {
                    setPathState(PathState.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (justEntered) {
                    follower.followPath(path4, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    robot.setOuttakeTwoPower(1);
                    if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SECONDS) {
                        robot.setOuttakeTwoPower(0);
                        setPathState(PathState.COLLECT_3);
                    }
                }
                break;

            case COLLECT_3:
                if (justEntered) {
                    follower.followPath(path5, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_4);
                }
                break;

            case SHOOT_4:
                if (justEntered) {
                    follower.followPath(line6, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    robot.setOuttakeTwoPower(1);
                    if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SECONDS) {
                        robot.setOuttakeTwoPower(0);
                        setPathState(PathState.LEAVE);
                    }
                }
                break;

            case LEAVE:
                if (justEntered) {
                    follower.followPath(path9, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
            default:
                telemetry.addLine("Autonomous Finished");
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        robot = new LancersRobot(hardwareMap, telemetry, true, false, true);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        setPathState(PathState.INITIAL_SHOOT);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }
}
