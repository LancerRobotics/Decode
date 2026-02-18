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

@Autonomous(name = "Blue15")
public class Blue15 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private LancersRobot robot;

    public enum PathState {
        STARTPOS_TO_SHOOTPOS,
        SHOOT,
        COLLECT,
        WAIT_AFTER_GATE,
        SHOOTPOS_TO_LEAVEPOS,
        DONE
    }

    private PathState pathState;
    private boolean justEntered = true;

    private int cycleIndex = 0;

    private final Pose startPose = new Pose(20, 122, Math.toRadians(136));
    private final Pose shootPose = new Pose(48, 96, Math.toRadians(136));
    private final Pose leavePose = new Pose(32, 80, Math.toRadians(136));
    private final Pose gatePose = new Pose(11.5, 58.5, Math.toRadians(145));

    private PathChain startPosToShootPos, shootPosToLeavePos;
    private PathChain collectFirstBalls, collectSecondBalls, collectThirdBalls;
    private PathChain shootPosToGatePos, gatePosToShootPos;

    private static final double SHOOT_SECONDS = 1.5;

    public void buildPaths() {

        startPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootPosToLeavePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .build();

        collectFirstBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(48.000, 92.000)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierCurve(
                        new Pose(48.000, 92.000),
                        new Pose(49.941, 82.414),
                        new Pose(43.000, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(43.000, 84.000),
                        new Pose(36.000, 84.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(36, 84),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();

        collectSecondBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(48.000, 92.000)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierCurve(
                        new Pose(48.000, 92.000),
                        new Pose(49.941, 58),
                        new Pose(43.000, 60.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(43.000, 60.000),
                        new Pose(30, 60.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(30, 60),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();

        collectThirdBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(48.000, 42.000)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(48, 42.000),
                        new Pose(30, 42.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Pose(30, 42),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();

        shootPosToGatePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(44, 58.5)))
                .setConstantHeadingInterpolation(Math.toRadians(136))
                .addPath(new BezierLine(new Pose(44, 58.5), gatePose))
                .setLinearHeadingInterpolation(Math.toRadians(136), gatePose.getHeading())
                .build();

        gatePosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, new Pose(44, 58.5)))
                .setLinearHeadingInterpolation(gatePose.getHeading(), Math.toRadians(136))
                .addPath(new BezierLine(new Pose(44, 58.5), shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(136))
                .build();
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        justEntered = true;
    }

    private PathChain getCollectPathForCycle(int cycleIndex) {
        switch (cycleIndex) {
            case 0: return collectSecondBalls;
            case 1: return shootPosToGatePos;
            case 2: return gatePosToShootPos;
            case 3: return collectFirstBalls;
            default: return collectThirdBalls;
        }
    }

    private void statePathUpdate() {
        switch (pathState) {

            case STARTPOS_TO_SHOOTPOS: {

                robot.setOuttakeVelocity(1500);
                robot.setIntake(1);

                if (justEntered) {
                    follower.followPath(startPosToShootPos, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    cycleIndex = 0;
                    setPathState(PathState.SHOOT);
                }
                break;
            }

            case SHOOT: {
                if (justEntered) {
                    justEntered = false;
                }

                robot.setOuttakeTwoPower(1);
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SECONDS) {
                    robot.setOuttakeTwoPower(0);
                    if (cycleIndex < 5) {
                        setPathState(PathState.COLLECT);
                    } else {
                        setPathState(PathState.SHOOTPOS_TO_LEAVEPOS);
                    }
                }
                break;
            }

            case COLLECT: {
                if (justEntered) {
                    follower.followPath(getCollectPathForCycle(cycleIndex), true);
                    justEntered = false;
                }

                if (!follower.isBusy()) {
                    if (cycleIndex == 1) {
                        cycleIndex++;
                        setPathState(PathState.COLLECT);
                    } else if (cycleIndex == 2) {
                        cycleIndex++;
                        setPathState(PathState.WAIT_AFTER_GATE);
                    } else {
                        cycleIndex++;
                        setPathState(PathState.SHOOT);
                    }
                }
                break;
            }

            case WAIT_AFTER_GATE: {
                if (justEntered) {
                    justEntered = false;
                }
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    setPathState(PathState.COLLECT);
                }
                break;
            }

            case SHOOTPOS_TO_LEAVEPOS: {
                if (justEntered) {
                    follower.followPath(shootPosToLeavePos, true);
                    justEntered = false;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;
            }

            case DONE:
            default: {
                telemetry.addLine("Auton Complete");
                break;
            }
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        robot = new LancersRobot(hardwareMap, telemetry, true, false, true);

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        setPathState(PathState.STARTPOS_TO_SHOOTPOS);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        justEntered = true;
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Cycle Index", cycleIndex);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
    }
}
