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

@Autonomous(name = "Red12")
public class Red12 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private LancersRobot robot;

    // --- STATE MACHINE ---
    public enum PathState {
        STARTPOS_TO_SHOOTPOS,
        SHOOT,
        COLLECT,
        SHOOTPOS_TO_LEAVEPOS,
        DONE
    }

    private PathState pathState;
    private boolean justEntered = true;

    // Cycle index for collecting and shooting
    private int cycleIndex = 0;

    // Relevant Poses
    private final Pose startPose = new Pose(124, 122, Math.toRadians(45));
    private final Pose shootPose = new Pose(96, 96, Math.toRadians(45));
    private final Pose leavePose = new Pose(112, 80, Math.toRadians(45));

    // Paths
    private PathChain startPosToShootPos, shootPosToLeavePos;
    private PathChain collectFirstBalls, collectSecondBalls, collectThirdBalls;

    // Timings
    private static final double SHOOT_SECONDS = 2.0; // TODO: tune later

    public void buildPaths() {

        startPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootPosToLeavePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .build();


        // Collect 1
        collectFirstBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(96.000, 92.000)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Pose(96.000, 92.000),
                        new Pose(111.076, 82.414),
                        new Pose(130, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(101.000, 84.000),
                        new Pose(126.11, 84.000)
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(126.11, 84),
                        shootPose
                )).setLinearHeadingInterpolation(Math.toRadians(0), shootPose.getHeading())
                .build();

        // Collect 2
        collectSecondBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(96.000, 92.000)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Pose(96.000, 92.000),
                        new Pose(94.059, 58),
                        new Pose(101, 60.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(101.000, 60.000),
                        new Pose(126.11, 60.000)
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(126.11, 60),
                        shootPose
                )).setLinearHeadingInterpolation(Math.toRadians(0), shootPose.getHeading())
                .build();

        // Collect 3
        collectThirdBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(96, 20.000)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Pose(96, 20.000),
                        new Pose(94.059, 58.414),
                        new Pose(101, 60.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(101, 60.000),
                        new Pose(126.11, 60.000)
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Pose(126.11, 60),
                        shootPose
                )).setLinearHeadingInterpolation(Math.toRadians(0), shootPose.getHeading())
                .build();
    }

    // Changes path state
    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        justEntered = true;
    }

    // Chooses the correct shooting and collecting path for the current cycle
    private PathChain getCollectPathForCycle(int cycleIndex) {
        switch (cycleIndex) {
            case 0: return collectFirstBalls;
            case 1: return collectSecondBalls;
            default: return collectThirdBalls;
        }
    }


    private void statePathUpdate() {
        switch (pathState) {

            case STARTPOS_TO_SHOOTPOS: {

                // start intake and outtake for the rest of the auton
                robot.setOuttakeVelocity(1500);
                robot.setIntake(1);

                if (justEntered) {
                    follower.followPath(startPosToShootPos, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    // arrived at shoot pose for the first time
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
                    if (cycleIndex < 3) {
                        setPathState(PathState.COLLECT); // Goes onto the next collect cycle
                    } else {
                        setPathState(PathState.SHOOTPOS_TO_LEAVEPOS); // Leaves
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
                    cycleIndex++; // Completed one cycle, going onto the next one
                    setPathState(PathState.SHOOT);
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

        robot = new LancersRobot(hardwareMap, telemetry);

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