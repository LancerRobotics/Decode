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
        START_TO_SHOOT,
        SHOOT_1,
        COLLECT_1_OUT,
        COLLECT_1_BACK,
        SHOOT_2,
        COLLECT_2_OUT,
        WAIT_GATE,
        COLLECT_2_BACK,
        SHOOT_3,
        COLLECT_3_OUT,
        COLLECT_3_BACK,
        SHOOT_4,
        COLLECT_4_OUT,
        COLLECT_4_BACK,
        SHOOT_5,
        PARK,
        DONE
    }

    private PathState pathState;
    private boolean justEntered = true;

    // Key poses
    private final Pose startPose    = new Pose(23.978, 122.736, Math.toRadians(138.630));
    private final Pose shootPose = new Pose(48,  96, Math.toRadians(135));

    private final Pose collect1Pose = new Pose(18.764,  59.357, Math.toRadians(180));
    private final Pose collect2Pose = new Pose( 8.909,  59.707, Math.toRadians(135));
    private final Pose collect3Pose = new Pose(16.140,  84.462, Math.toRadians(180));
    private final Pose collect4Pose = new Pose(16.160,  35.446, Math.toRadians(180));

    private final Pose parkPose     = new Pose(54.384, 107.059, Math.toRadians(180));

    // Path 1-8 match the JSON segments; path9 and parkPath split the original JSON Path 9
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, parkPath;

    private static final double SHOOT_SECONDS     = 1.0;
    private static final double GATE_WAIT_SECONDS = 4.0;

    public void buildPaths() {
        // Path 1: start -> shoot (BezierLine, constant 135°)
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        // Path 2: shoot -> collect1 (BezierCurve, linear 135->180°)
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(50.428, 53.484),
                        collect1Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 3: collect1 -> shoot (BezierCurve, linear 180->135°)
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        collect1Pose,
                        new Pose(38.174, 77.246),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        // Path 4: shoot -> collect2 (BezierCurve, constant 135°)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(38.615, 73.909),
                        collect2Pose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        // Path 5: collect2 -> shoot (BezierCurve, constant 135°)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        collect2Pose,
                        new Pose(47.442, 55.631),
                        shootPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        // Path 6: shoot -> collect3 (BezierCurve, linear 135->180°)
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(40.335, 82.340),
                        collect3Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 7: collect3 -> shoot (BezierLine, linear 180->180°)
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(collect3Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 8: shoot -> collect4 (BezierCurve, linear 180->180°)
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(50.205, 26.088),
                        collect4Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 9: collect4 -> final shoot position (BezierLine, constant 180°)
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(collect4Pose, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Park: final shoot position -> park (BezierLine, constant 180°)
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        justEntered = true;
    }

    private void statePathUpdate() {
        switch (pathState) {

            case START_TO_SHOOT: {
                robot.setOuttakeVelocity(1300);
                robot.setIntake(1);
                robot.setServoPosition(0.5);
                if (justEntered) {
                    follower.followPath(path1, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_1);
                }
                break;
            }

            case SHOOT_1:
            case SHOOT_2:
            case SHOOT_3:
            case SHOOT_4:
            case SHOOT_5: {
                if (justEntered) {
                    justEntered = false;
                }
                robot.setOuttakeTwoPower(1);
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_SECONDS) {
                    robot.setOuttakeTwoPower(0);
                    if      (pathState == PathState.SHOOT_1) setPathState(PathState.COLLECT_1_OUT);
                    else if (pathState == PathState.SHOOT_2) setPathState(PathState.COLLECT_2_OUT);
                    else if (pathState == PathState.SHOOT_3) setPathState(PathState.COLLECT_3_OUT);
                    else if (pathState == PathState.SHOOT_4) setPathState(PathState.COLLECT_4_OUT);
                    else                                     setPathState(PathState.PARK);
                }
                break;
            }

            case COLLECT_1_OUT: {
                if (justEntered) {
                    follower.followPath(path2, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.COLLECT_1_BACK);
                }
                break;
            }

            case COLLECT_1_BACK: {
                if (justEntered) {
                    follower.followPath(path3, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_2);
                }
                break;
            }

            case COLLECT_2_OUT: {
                if (justEntered) {
                    follower.followPath(path4, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_GATE);
                }
                break;
            }

            case WAIT_GATE: {
                if (justEntered) {
                    justEntered = false;
                }
                if (pathTimer.getElapsedTimeSeconds() >= GATE_WAIT_SECONDS) {
                    setPathState(PathState.COLLECT_2_BACK);
                }
                break;
            }

            case COLLECT_2_BACK: {
                if (justEntered) {
                    follower.followPath(path5, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_3);
                }
                break;
            }

            case COLLECT_3_OUT: {
                if (justEntered) {
                    follower.followPath(path6, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.COLLECT_3_BACK);
                }
                break;
            }

            case COLLECT_3_BACK: {
                if (justEntered) {
                    follower.followPath(path7, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_4);
                }
                break;
            }

            case COLLECT_4_OUT: {
                if (justEntered) {
                    follower.followPath(path8, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.COLLECT_4_BACK);
                }
                break;
            }

            case COLLECT_4_BACK: {
                if (justEntered) {
                    follower.followPath(path9, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_5);
                }
                break;
            }

            case PARK: {
                if (justEntered) {
                    follower.followPath(parkPath, true);
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

        setPathState(PathState.START_TO_SHOOT);
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
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
    }
}