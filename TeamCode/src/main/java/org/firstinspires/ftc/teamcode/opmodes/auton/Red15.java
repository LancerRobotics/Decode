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

@Autonomous(name = "Red15")
public class Red15 extends OpMode {

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

    // TODO: tune these two velocities
    private static final double CLOSE_SHOT_VELOCITY = 960;  // TODO: tune close shot ticks/sec
    private static final double FAR_SHOT_VELOCITY   = 1240; // TODO: tune far shot ticks/sec

    private final Pose startPose    = new Pose(144-20.477189627228526, 122.736, Math.toRadians(180-144));
    private final Pose collect1Pose = new Pose(144-18.764,  59.707, Math.toRadians(180-135));
    private final Pose collect2Pose = new Pose(144-8.909,   59.707, Math.toRadians(180-135));
    private final Pose collect3Pose = new Pose(144-16.140,  84.462, Math.toRadians(180-180));
    private final Pose collect4Pose = new Pose(144-16.160,  35.446, Math.toRadians(180-180));
    private final Pose parkPose     = new Pose(144-54.384, 107.059, Math.toRadians(180-180));

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    private static final double GATE_WAIT_SECONDS = 4.0;

    public void buildPaths() {
        path1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(120.498, 120.871), new Pose(96.311, 94.899))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

        path2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(96.311, 94.899), new Pose(95.844, 55.251), new Pose(131.387, 58.664))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

        path3 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(131.387, 58.664), new Pose(100.330, 66.737), new Pose(95.938, 94.381))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

        path4 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(95.938, 94.381), new Pose(112.603, 65.851), new Pose(134.227, 60.998))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45)).build();

        path5 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(134.227, 60.998), new Pose(108.999, 70.700), new Pose(96.521, 94.348))
        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0)).build();

        path6 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(96.521, 94.348), new Pose(84.152, 28.981), new Pose(131.114, 35.308))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

        path7 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(131.114, 35.308), new Pose(96.610, 93.597))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

        path8 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(96.610, 93.597), new Pose(107.691, 80.607), new Pose(131.376, 84.126))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

        path9 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(131.376, 84.126), new Pose(108.478, 81.081), new Pose(88.028, 107.837))
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        justEntered = true;
    }

    private void statePathUpdate() {
        switch (pathState) {

            case START_TO_SHOOT: {
                if (justEntered) {
                    follower.followPath(path1, true);
                    justEntered = false;
                }
                if (!follower.isBusy()) setPathState(PathState.SHOOT_1);
                break;
            }

            case SHOOT_1:
            case SHOOT_2:
            case SHOOT_3:
            case SHOOT_4:
            case SHOOT_5: {
                if (justEntered) {
                    robot.setOuttakeTwoPower(1);
                    justEntered = false;
                }
                robot.setOuttakeTwoPower(-1);
                if      (pathState == PathState.SHOOT_1) setPathState(PathState.COLLECT_1_OUT);
                else if (pathState == PathState.SHOOT_2) setPathState(PathState.COLLECT_2_OUT);
                else if (pathState == PathState.SHOOT_3) setPathState(PathState.COLLECT_3_OUT);
                else if (pathState == PathState.SHOOT_4) setPathState(PathState.COLLECT_4_OUT);
                else                                     setPathState(PathState.PARK);
                break;
            }

            case COLLECT_1_OUT: {
                if (justEntered) { follower.followPath(path2, true); justEntered = false; }
                if (!follower.isBusy()) setPathState(PathState.COLLECT_1_BACK);
                break;
            }

            case COLLECT_1_BACK: {
                if (justEntered) { follower.followPath(path3, true); justEntered = false; }
                if (!follower.isBusy()) setPathState(PathState.SHOOT_2);
                break;
            }

            case COLLECT_2_OUT: {
                if (justEntered) { follower.followPath(path4, true); justEntered = false; }
                if (!follower.isBusy()) setPathState(PathState.WAIT_GATE);
                break;
            }

            case WAIT_GATE: {
                if (justEntered) justEntered = false;
                if (pathTimer.getElapsedTimeSeconds() >= GATE_WAIT_SECONDS) setPathState(PathState.COLLECT_2_BACK);
                break;
            }

            case COLLECT_2_BACK: {
                if (justEntered) { follower.followPath(path5, true); justEntered = false; }
                if (!follower.isBusy()) setPathState(PathState.SHOOT_3);
                break;
            }

            case COLLECT_3_OUT: {
                if (justEntered) { follower.followPath(path6, true); justEntered = false; }
                if (!follower.isBusy()) setPathState(PathState.COLLECT_3_BACK);
                break;
            }

            case COLLECT_3_BACK: {
                if (justEntered) { follower.followPath(path7, true); justEntered = false; }
                if (!follower.isBusy()) setPathState(PathState.SHOOT_4);
                break;
            }

            case COLLECT_4_OUT: {
                if (justEntered) { follower.followPath(path8, true); justEntered = false; }
                if (!follower.isBusy()) setPathState(PathState.COLLECT_4_BACK);
                break;
            }

            case COLLECT_4_BACK: {
                if (justEntered) { follower.followPath(path9, true); justEntered = false; }
                if (!follower.isBusy()) setPathState(PathState.SHOOT_5);
                break;
            }

            case PARK: {
                setPathState(PathState.DONE);
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

        robot = new LancersRobot(hardwareMap, telemetry, true, true, true);
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

        // Always-on motors — start at close shot velocity by default
        robot.setOuttakePIDF(robot.closeShootingPIDF);
        robot.setOuttakeVelocity(CLOSE_SHOT_VELOCITY); // TODO: tune close shot ticks/sec
        robot.setIntake(1);
        robot.setServoPosition(0.5);
        robot.setOuttakeTwoPower(-1);
    }

    @Override
    public void loop() {
        follower.update();

        // Turret always tracking the red goal
        robot.aimTurretToAngle(robot.getIntegratedAngle(true), 0.8, 5);

        statePathUpdate();

        // ONE telemetry.update() at the very end
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Turret Angle", robot.getCurrentTurretDeg());
        telemetry.addData("Target Angle", robot.getIntegratedAngle(true));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}